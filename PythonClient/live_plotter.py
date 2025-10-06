#!/usr/bin/python3

import tkinter as tk
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.backends.backend_agg import FigureCanvasAgg
import pygame

class Dynamic2DFigure():
    def __init__(self, 
                 figsize=(8,8), 
                 edgecolor="black", 
                 rect=[0.1, 0.1, 0.8, 0.8],
                 *args, **kwargs):
        self.graphs = {}
        self.texts = {}
        self.fig = plt.Figure(figsize=figsize, edgecolor=edgecolor)
        self.ax = self.fig.add_axes(rect)
        self.fig.tight_layout()
        self.marker_text_offset = 0
        if kwargs.get("title") is not None:
            self.fig.suptitle(kwargs["title"])
        self.axis_equal = False
        self.invert_xaxis = False

    def set_invert_x_axis(self):
        self.invert_xaxis = True

    def set_axis_equal(self):
        self.axis_equal = True

    def add_graph(self, name, label="", window_size=10, x0=None, y0=None,
                  linestyle='-', linewidth=1, marker="", color="k", 
                  markertext=None, marker_text_offset=2):
        self.marker_text_offset = marker_text_offset
        if x0 is None or y0 is None:
            x0 = np.zeros(window_size)
            y0 = np.zeros(window_size)
        new_graph, = self.ax.plot(x0, y0, label=label, 
                                  linestyle=linestyle, linewidth=linewidth,
                                  marker=marker, color=color)
        if markertext is not None:
            new_text = self.ax.text(x0[-1], y0[-1] + marker_text_offset, markertext)
            self.texts[name + "_TEXT"] = new_text
        self.graphs[name] = new_graph

    def roll(self, name, new_x, new_y):
        graph = self.graphs[name]
        if graph is not None:
            x, y = graph.get_data()
            x = np.roll(x, -1)
            x[-1] = new_x
            y = np.roll(y, -1)
            y[-1] = new_y
            graph.set_data((x, y))
            self.rescale()
        if name + "_TEXT" in self.texts:
            graph_text = self.texts[name + "_TEXT"]
            graph_text.set_position((new_x, new_y + self.marker_text_offset))
            self.rescale()

    def update(self, name, new_x_vec, new_y_vec, new_colour='k'):
        graph = self.graphs[name]
        if graph is not None:
            graph.set_data((np.array(new_x_vec), np.array(new_y_vec)))
            graph.set_color(new_colour)
            self.rescale()
        if name + "_TEXT" in self.texts:
            graph_text = self.texts[name + "_TEXT"]
            x = new_x_vec[-1]
            y = new_y_vec[-1] + self.marker_text_offset
            graph_text.set_position((x, y))
            self.rescale()

    def rescale(self):
        xmin = float("inf")
        xmax = -1*float("inf")
        ymin, ymax = self.ax.get_ylim()
        for name, graph in self.graphs.items():
            xvals, yvals = graph.get_data()
            xmin_data = xvals.min()
            xmax_data = xvals.max()
            ymin_data = yvals.min()
            ymax_data = yvals.max()
            xmin_padded = xmin_data-0.05*(xmax_data-xmin_data)
            xmax_padded = xmax_data+0.05*(xmax_data-xmin_data)
            ymin_padded = ymin_data-0.05*(ymax_data-ymin_data)
            ymax_padded = ymax_data+0.05*(ymax_data-ymin_data)
            xmin = min(xmin_padded, xmin)
            xmax = max(xmax_padded, xmax)
            ymin = min(ymin_padded, ymin)
            ymax = max(ymax_padded, ymax)
        self.ax.set_xlim(xmin, xmax)
        self.ax.set_ylim(ymin, ymax)
        if self.axis_equal:
            self.ax.set_aspect('equal')
        if self.invert_xaxis:
            self.ax.invert_xaxis()


class DynamicFigure():
    def __init__(self, *args, **kwargs):
        self.graphs = {}
        self.fig = plt.Figure(figsize=(3, 2), edgecolor="black")
        self.ax = self.fig.add_axes([0.2, 0.2, 0.6, 0.6])
        self.fig.tight_layout()
        if kwargs.get("title") is not None:
            self.fig.suptitle(kwargs["title"])

    def add_graph(self, name, label="", window_size=10, x0=None, y0=None):
        if y0 is None:
            x0 = np.zeros(window_size)
            y0 = np.zeros(window_size)
        new_graph, = self.ax.plot(x0, y0, label=label)
        self.graphs[name] = new_graph

    def roll(self, name, new_x, new_y):
        graph = self.graphs[name]
        if graph is not None:
            x, y = graph.get_data()
            x = np.roll(x, -1)
            x[-1] = new_x
            y = np.roll(y, -1)
            y[-1] = new_y
            graph.set_data((x, y))
            self.rescale()

    def rescale(self):
        xmin = float("inf")
        xmax = -1*float("inf")
        ymin, ymax = self.ax.get_ylim()
        for name, graph in self.graphs.items():
            xvals, yvals = graph.get_data()
            xmin_data = xvals.min()
            xmax_data = xvals.max()
            ymin_data = yvals.min()
            ymax_data = yvals.max()
            xmin_padded = xmin_data-0.05*(xmax_data-xmin_data)
            xmax_padded = xmax_data+0.05*(xmax_data-xmin_data)
            ymin_padded = ymin_data-0.05*(ymax_data-ymin_data)
            ymax_padded = ymax_data+0.05*(ymax_data-ymin_data)
            xmin = min(xmin_padded, xmin)
            xmax = max(xmax_padded, xmax)
            ymin = min(ymin_padded, ymin)
            ymax = max(ymax_padded, ymax)
        self.ax.set_xlim(xmin, xmax)
        self.ax.set_ylim(ymin, ymax)


class LivePlotter():
    def __init__(self, tk_title=None):
        self._display = None
        self._game_frame = None
        self._pygame_init = False
        self._surfs = []
        self._surf_coords = {}
        self._figs = []
        self._fcas = {}
        self._root = tk.Tk()
        if tk_title is not None:
            self._root.title(tk_title)
        self._canvas = tk.Canvas(self._root, width=150, height=100, bg="#6A6A6A")
        self._canvas.grid(row=0, column=0)
        self._empty = True
        self._text_id = self._canvas.create_text(75, 50, text="No live plots\ncreated yet.")

    def plot_figure(self, fig):
        if self._empty:
            self._empty = False
            self._canvas.delete(self._text_id)
        canvas = FigureCanvasTkAgg(fig, master=self._canvas)
        canvas.draw()
        canvas.get_tk_widget().pack()
        self._figs.append(fig)
        self._fcas[fig] = canvas

    def plot_new_dynamic_figure(self, title=""):
        dyfig = DynamicFigure(title=title)
        fig = dyfig.fig
        self.plot_figure(fig)
        return dyfig

    def plot_new_dynamic_2d_figure(self, title="", **kwargs):
        dy2dfig = Dynamic2DFigure(title=title, **kwargs)
        fig = dy2dfig.fig
        self.plot_figure(fig)
        return dy2dfig

    def refresh_figure(self, fig):
        self._fcas[fig].draw()
        self._root.update()

    def refresh(self):
        for fig in list(self._figs):
            self.refresh_figure(fig)
        if self._display is not None:
            self._display.blits(list(self._surf_coords.items()))
            pygame.display.flip()
