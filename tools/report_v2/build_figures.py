import figures, figures_flow, figures_eval
for fn in (figures.fig_physics, figures.fig_arch, figures.fig_codesize,
           figures.fig_callgraph, figures_flow.fig_pipeline, figures_flow.fig_gate,
           figures_flow.fig_forward, figures_flow.fig_defense,
           figures_eval.fig_dvf, figures_eval.fig_perf, figures_eval.fig_test,
           figures_eval.fig_status, figures_eval.fig_complete,
           figures_eval.fig_limits, figures_eval.fig_timeline):
    fn()
print("svgs:", len(list(figures.OUT.glob("*.svg"))))
