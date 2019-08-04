import plotly.graph_objs as go
from plotly.offline import iplot, init_notebook_mode

init_notebook_mode()

def plot_dqn_cost(total_costs, known_costs):
    epoch_idx = list(range(len(total_costs)))
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=epoch_idx, y=total_costs, name='total_cost'))
    fig.add_trace(go.Scatter(x=epoch_idx, y=known_costs, name='known_cost'))
    fig.layout.update(
        title="Total and Known Cost",
        xaxis=dict(title="Epoch index"),
        yaxis=dict(title="Total cost, negative")
    )
    iplot(fig)
