
import matplotlib.pyplot as plt

def edit_figure_for_web(ax, leg):
  mycolor = 'w'

  # makes all axes and text whie
  for l in ['bottom', 'left', 'right', 'top']:
    ax.spines[l].set_color(mycolor)

  ax.xaxis.label.set_color(mycolor);
  ax.tick_params(axis='x', colors=mycolor)
  ax.yaxis.label.set_color(mycolor);
  ax.tick_params(axis='y', colors=mycolor)

  for text in leg.get_texts(): text.set_color(mycolor)
