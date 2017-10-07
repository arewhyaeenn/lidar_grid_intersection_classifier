from tkinter import Tk, Canvas, LAST, Label, Text

class Graphics:
    def __init__(self, w, h):
        """
        Creates a window which is used to draw the scene.
        Parameters are width and height (px).
        :param w: width of the window
        :param h: height of the window
        """
        self.active = True
        self.win = Tk()
        self.win.wm_title('Bot simulation')
        self.win.protocol("WM_DELETE_WINDOW", lambda: self.stop())

        self.width = w
        self.height = h
        self.canvas = Canvas(self.win, width=self.width, height=self.height)
        self.canvas.pack()

        self.win.bind('<Button-1>', self.pressed)
        self.win.bind('<Double-1>', self.clicked)

        self.label = Label(self.win,
              text='Coordinates',
              fg='dark blue',
              font='Verdana 18 bold')
        self.label.pack()

        self.text_entry = Text(self.win, height=2, width=30)
        self.text_entry.pack()

        # self.win.mainloop()

    def animate(self, wall_map, bots):
        """
        Draws one frame of animation based on the state of the bots and walls passed in.
        :param wall_map: the maze as a list of np.arrays that represent line segments (walls)
        :param bots: a list of bots in the maze
        """
        self.canvas.delete('all')
        # animate walls
        for wall in wall_map.walls:
            self.canvas.create_line(tuple(wall.reshape([1, 4])[0]))

        # animate bots
        for bot in bots:
            self.canvas.create_text(
                tuple(bot.pos + bot.rad),
                anchor='nw',
                fill=bot.color,
                text='x:{:3.0f}\ny:{:3.0f}\n'.format(bot.pos[0], bot.pos[1]) + bot.quote
            )
            self.canvas.create_line(
                tuple(bot.pos),
                tuple(bot.pos + (bot.rad * bot.dir)),
                arrow=LAST,
                fill=bot.color
            )
            self.canvas.create_oval(
                tuple(bot.pos - bot.rad),
                tuple(bot.pos + bot.rad),
                outline=bot.color
            )
            # animate lidar
            for i in range(0, len(bot.scan_points)):
                self.canvas.create_line(
                    tuple(bot.pos),
                    tuple(bot.scan_points[i]),
                    dash=(1, bot.rad // 2),
                    fill=bot.color
                )

        self.win.update()

    def stop(self):
        self.active = False

    def end(self):
        self.win.destroy()

    def pressed(self, event):
        self.label.config(text='Pressed at position: ({} {})'.format(event.x, event.y))

    def clicked(self, event):
        self.label.config(text='Pressed at position: ({} {})'.format(event.x, event.y))


