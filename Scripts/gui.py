#!/usr/bin/env python
import PySimpleGUI27 as sg

# Usage of Graph element.

layout = [[sg.Graph(canvas_size=(400, 400), graph_bottom_left=(-200, -200), graph_top_right=(200, 200), background_color='red', key='graph', enable_events=True, drag_submits=True)]]

window = sg.Window('Graph test', layout, finalize=True)

graph = window['graph']

pointSize = 10

oval1 = graph.draw_oval((-5, 0), (5, 50), fill_color='purple', line_color='purple')
oval3 = graph.draw_oval((0, 5), (50, -5), fill_color='purple', line_color='purple')
oval2 = graph.draw_oval((5, 0), (-5, -50), fill_color='blue', line_color='blue')
oval4 = graph.draw_oval((0, -5), (-50, 5), fill_color='blue', line_color='blue')
circle2 = graph.draw_circle((0, 0), 15, fill_color='black', line_color='green')

point = None

while True:
	event, values = window.Read(timeout=25)
	print(event, values)

	if event is None:
		break

	if not event == u'__TIMEOUT__':
		_x, _y = values["graph"]
		if point is None:
			point = graph.draw_point( [_x - pointSize, _y + pointSize], pointSize, color='green')
		else:
			graph.RelocateFigure(point, _x - pointSize, _y + pointSize)





	# if event in ('Blue', 'Red'):
	# 	graph.TKCanvas.itemconfig(circle, fill=event)
	# elif event == 'Move':
	# 	graph.Position
	# 	graph.MoveFigure(circle, 10, 10)
	# 	graph.MoveFigure(oval, 10, 10)
	# 	graph.MoveFigure(rectangle, 10, 10)
	# 	graph.MoveFigure(arc, 10, 10)

window.close()