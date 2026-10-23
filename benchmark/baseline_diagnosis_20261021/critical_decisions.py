"""Summarize two independently captured D01 critical cycles without new scoring."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE=Path(__file__).resolve().parent


def main():
    summaries=json.loads((HERE/'outputs/score_summary.json').read_text())
    result=[]
    fig,axs=plt.subplots(1,2,figsize=(12,4.8),layout='constrained')
    for ax,summary in zip(axs,summaries[:2]):
        c=summary['representative'];directory=Path(summary['source'])
        records=[json.loads(line) for line in next(directory.glob('decisions_*.jsonl')).read_text().splitlines()]
        original=records[c['cycle']];g=original['costmap'];arr=np.array(g['data']).reshape(g['height'],g['width'])
        x=g['origin'][0]+(np.arange(g['width'])+.5)*g['resolution']
        y=g['origin'][1]+(np.arange(g['height'])+.5)*g['resolution']
        roi=(x[None,:]>=1.15)&(x[None,:]<=2.05)&(y[:,None]>=-1.4)&(y[:,None]<=1.4)
        iy,ix=np.where(roi&(arr==254))
        selected=c['key_candidates']['selected'];slow=c['key_candidates']['slowest_best']
        # Bounds, not a replacement score: descriptive Top-K cannot exceed max.
        bound=summary['weights'][0]*(selected['max']-selected['mean'])/252.
        row=dict(run=summary['run'],t=c['t'],input_stamp=c['input_stamp'],selected=selected,
            slowest_best=slow,weighted_delta_slowest_minus_selected=c['slowest_minus_selected'],
            selected_obstacle_mean_to_peak_contribution_bound=bound,
            lethal_cells_in_fixed_prior_roi=len(ix),
            lethal_cell_centers_in_fixed_prior_roi=[[float(x[i]),float(y[j])] for j,i in zip(iy,ix)],
            roi=[1.15,-1.4,2.05,1.4],
            contact_source_relative_s=summary['contact_source_relative_s'],
            conditional_safe_key_source='Existing avoidability conditional model; not a physical last-safe-time proof.',
            map_timestamp=None,map_time_limit='Locked decision snapshot, not a stamped map publication.')
        result.append(row)
        bottom=np.zeros(2)
        colors=['#b44c43','#f3ad48','#476d9e','#62a08b','#9b76ae']
        for term,color in zip('OSGPH',colors):
            values=np.array([selected['weighted_'+term],slow['weighted_'+term]])
            ax.bar([0,1],values,bottom=bottom,label=term,color=color,width=.65);bottom+=values
        ax.set_xticks([0,1],['selected\nv=%.3f'%selected['v'],'lowest feasible v\nv=%.3f'%slow['v']])
        ax.set_title(summary['run']+f"\nt={c['t']:.3f}s; current selected mean={selected['mean']:.2f}")
        ax.set_ylabel('original weighted cost');ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
        for i,total in enumerate(bottom):ax.text(i,total+.005,f'{total:.4f}',ha='center')
        ax.set_ylim(0,.4)
    axs[1].legend(title='original terms',ncol=5,loc='upper right')
    fig.suptitle('Actual score decomposition — no Top-K score or policy executed')
    fig.savefig(HERE/'outputs/figures/critical_weighted_contributions.png',dpi=160);plt.close(fig)
    (HERE/'outputs/critical_decisions.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print([(r['run'],r['t'],r['lethal_cells_in_fixed_prior_roi']) for r in result])


if __name__=='__main__':main()
