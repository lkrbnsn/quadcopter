from Tkinter import *

roll = 0

def kill():
	# TODO kill quad
	print("hello")

def update():
	# TODO send data to quad
	print(altitudeScale.get(), rollScale.get(), pitchScale.get())
	master.after(1000, update)

def leftKey(event):
    print "Left key pressed"
    roll = roll + 1
    rollScale.set(roll)

def rightKey(event):
    print "Right key pressed"


##############################################
# This is where the program begins execution

# Create a window
master = Tk()

# Title bar of the window
master.wm_title("Quadcopter Control System")


# warningLabel = Label(master, text="Controller", font=(14))
# warningLabel.grid(row=1, column=1)

altitudeScale = Scale(master, from_=80, to=50)
altitudeScale.grid(row=1, column=1)

rollScale = Scale(master, from_=10, to=-10, orient=HORIZONTAL)
rollScale.grid(row=1, column=2, padx=10)

pitchScale = Scale(master, from_=10, to=-10)
pitchScale.grid(row=1, column=3)

# Insert kill button
refreshButton = Button(master, text="Stop", command=kill, font=(14))
refreshButton.grid(row=2, column=2, pady=20)

master.bind('<Left>', leftKey)
master.bind('<Right>', rightKey)

# TODO make this faster
master.after(1000, update)

# Open the window
mainloop()