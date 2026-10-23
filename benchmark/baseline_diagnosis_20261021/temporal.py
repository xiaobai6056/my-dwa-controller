"""Source-stamped rolling OccupancyGrid temporal evidence (offline only).

No truth fields enter extraction. Axis-aligned origins are required: observer
serializes x/y but no origin orientation. Cells are registered to the first
map's odom cell centers; only known common coverage may add/remove lethal=100.
99 (inscribed), inflation, unknown transitions and new/left coverage are separate.
"""
import hashlib
import json
import math


def centroid(points):
    return [sum(p[k] for p in points)/len(points) for k in (0,1)] if points else None


class History:
    def __init__(self,roi=None,max_gap=1.0,min_changed=2,min_displacement=.02):
        self.roi=roi;self.max_gap=max_gap;self.min_changed=min_changed
        self.min_displacement=min_displacement
        self.unique_count=0;self.previous=None;self.previous_result=None
        self.reference=None;self.seen={};self.content_seen=set()

    def feed(self,row):
        stamp=float(row['stamp']);receipt=float(row['t']);res=float(row['resolution'])
        if not all(math.isfinite(v) for v in (stamp,receipt,res,*row['origin'])) or res<=0:
            raise ValueError('invalid time/resolution/origin')
        if abs(float(row.get('origin_yaw',0.)))>1e-10:raise ValueError('unsupported origin orientation')
        if row.get('frame')!='odom':raise ValueError('unsupported initial/current frame; require odom')
        width,height=int(row['width']),int(row['height'])
        if width<=0 or height<=0 or len(row['data'])!=width*height:
            raise ValueError('invalid dimensions')
        if any(int(v)!=v or not -1<=v<=100 for v in row['data']):raise ValueError('invalid OccupancyGrid value')
        signature=hashlib.sha256(json.dumps([row['frame'],res,row['origin'],width,height,row['data']],separators=(',',':')).encode()).hexdigest()
        if stamp in self.seen:
            if self.seen[stamp]!=signature:raise ValueError('conflicting repeat source stamp')
            return dict(status='duplicate_stamp',stamp=stamp,receipt=receipt)
        if self.previous and stamp<self.previous['stamp']:raise ValueError('backward source stamp')
        if self.reference is None:
            ref=(row['frame'],res,list(row['origin']))
        else:ref=self.reference
        if row['frame']!=ref[0]:raise ValueError('unsupported frame change')
        if abs(res-ref[1])>1e-10:raise ValueError('unsupported resolution change')
        offsets=[(row['origin'][i]-ref[2][i])/res for i in (0,1)]
        if any(abs(v-round(v))>1e-5 for v in offsets):raise ValueError('nonintegral rolling origin shift')
        ix,iy=map(round,offsets)
        cells={(ix+x,iy+y):int(row['data'][y*width+x]) for y in range(height) for x in range(width)}
        def xy(k):return [ref[2][0]+(k[0]+.5)*res,ref[2][1]+(k[1]+.5)*res]
        def in_roi(k):
            x,y=xy(k)
            return self.roi is None or self.roi[0]<=x<=self.roi[2] and self.roi[1]<=y<=self.roi[3]
        current=dict(stamp=stamp,receipt=receipt,cells=cells,signature=signature)
        result=dict(status='initial' if self.previous is None else 'pair',stamp=stamp,receipt=receipt,
                    frame=row['frame'],origin=row['origin'],resolution=res,
                    repeated_content=signature in self.content_seen,total_cells=len(cells),
                    unknown_n=sum(v<0 for v in cells.values()),lethal_n=sum(v==100 for v in cells.values()),
                    inscribed_n=sum(v==99 for v in cells.values()),roi=self.roi,
                    roi_lethal_xy=[xy(k) for k,v in cells.items() if v==100 and in_roi(k)])
        if self.previous:
            old=self.previous['cells'];overlap=old.keys()&cells.keys()
            if not overlap:raise ValueError('no common coverage overlap')
            known={k for k in overlap if old[k]>=0 and cells[k]>=0}
            region={k for k in known if in_roi(k)}
            added=sorted(k for k in region if old[k]!=100 and cells[k]==100)
            removed=sorted(k for k in region if old[k]==100 and cells[k]!=100)
            before=[xy(k) for k in region if old[k]==100]
            after=[xy(k) for k in region if cells[k]==100]
            c0,c1=centroid(before),centroid(after)
            displacement=[c1[i]-c0[i] for i in (0,1)] if c0 and c1 else None
            norm=math.hypot(*displacement) if displacement else None
            dt=stamp-self.previous['stamp'];gap=dt>self.max_gap+1e-9
            candidate=bool(not gap and len(added)>=self.min_changed and len(removed)>=self.min_changed and norm is not None and norm>=self.min_displacement)
            prior=self.previous_result
            sustained=False;support=[self.previous['stamp'],stamp]
            if candidate and prior and prior.get('motion_candidate'):
                a,b=prior['centroid_displacement_xy'],displacement
                cosine=sum(a[i]*b[i] for i in (0,1))/(math.hypot(*a)*math.hypot(*b))
                sustained=cosine>=.5
                if sustained:support=[prior['previous_stamp'],self.previous['stamp'],stamp]
            # Scenario-axis evidence is a predeclared geometric filter, independent of GT.
            # A full grid-cell positive y displacement must dominate lateral displacement.
            axis_candidate=bool(candidate and displacement[1]>=res-1e-9 and displacement[1]>=abs(displacement[0]))
            axis_sustained=bool(axis_candidate and prior and prior.get('axis_motion_candidate'))
            axis_support=[prior['previous_stamp'],self.previous['stamp'],stamp] if axis_sustained else [self.previous['stamp'],stamp]
            result.update(axis_motion_candidate=axis_candidate,sustained_axis_direction=axis_sustained,
                          axis_support_stamps=axis_support,axis_rule='positive y displacement >= one cell and >= abs(dx); >=2 add/2 remove',
                          previous_stamp=self.previous['stamp'],previous_receipt=self.previous['receipt'],
                          dt_s=dt,gap=gap,overlap_n=len(overlap),known_overlap_n=len(known),roi_known_overlap_n=len(region),
                          known_to_unknown_n=sum(old[k]>=0 and cells[k]<0 for k in overlap),
                          unknown_to_known_n=sum(old[k]<0 and cells[k]>=0 for k in overlap),
                          entered_coverage_n=len(cells.keys()-old.keys()),left_coverage_n=len(old.keys()-cells.keys()),
                          entered_lethal_n=sum(cells[k]==100 for k in cells.keys()-old.keys()),
                          left_lethal_n=sum(old[k]==100 for k in old.keys()-cells.keys()),
                          inscribed_before_n=sum(old[k]==99 for k in region),inscribed_after_n=sum(cells[k]==99 for k in region),
                          added_n=len(added),removed_n=len(removed),added_xy=[xy(k) for k in added],removed_xy=[xy(k) for k in removed],
                          centroid_before_xy=c0,centroid_after_xy=c1,centroid_displacement_xy=displacement,
                          displacement_m=norm,motion_candidate=candidate,sustained_direction=sustained,support_stamps=support,
                          direction_xy=[v/norm for v in displacement] if candidate else None,
                          additions_centroid_xy=centroid([xy(k) for k in added]),removals_centroid_xy=centroid([xy(k) for k in removed]))
        self.reference=ref;self.previous=current;self.previous_result=result
        self.seen[stamp]=signature;self.content_seen.add(signature);self.unique_count+=1
        return result


def availability(rows,decision_s,receipt_margin=.1):
    """Conservative observer-side availability, NOT controller ingestion proof.

    Receipt t is quantized /clock. Add 0.1s before calling a map available;
    require source stamp <= decision as well. Exact arrival order is unknown.
    """
    eligible=[r for r in rows if r['status'] in ('initial','pair') and
              r['stamp']<=decision_s+1e-9 and r['receipt']+receipt_margin<=decision_s+1e-9]
    motion=next((r for r in eligible if r.get('motion_candidate')),None)
    axis=next((r for r in eligible if r.get('axis_motion_candidate')),None)
    axis_sustained=next((r for r in eligible if r.get('sustained_axis_direction')),None)
    sustained=next((r for r in eligible if r.get('sustained_direction')),None)
    def lead(r):return decision_s-max(r['stamp'],r['receipt']+receipt_margin) if r else None
    return dict(axis_motion_pair_lead_s=lead(axis),axis_direction_lead_s=lead(axis_sustained),
                available_axis_motion_pairs=sum(bool(r.get('axis_motion_candidate')) for r in eligible),
                available_axis_direction_pairs=sum(bool(r.get('sustained_axis_direction')) for r in eligible),
                decision_s=decision_s,receipt_margin_s=receipt_margin,
                available_unique_maps=len(eligible),available_motion_pairs=sum(bool(r.get('motion_candidate')) for r in eligible),
                available_sustained_direction=sum(bool(r.get('sustained_direction')) for r in eligible),
                motion_pair_lead_s=lead(motion),sustained_direction_lead_s=lead(sustained),
                last_source_stamp=eligible[-1]['stamp'] if eligible else None,
                first_motion_support_stamps=motion.get('support_stamps') if motion else None,
                first_direction_support_stamps=sustained.get('support_stamps') if sustained else None)
