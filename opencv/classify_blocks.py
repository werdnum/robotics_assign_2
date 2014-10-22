#!/usr/bin/env python

from Tkinter import Tk, PhotoImage, Label
from PIL import Image

class classifier(Tk):
	def __init__(self, parent):
		Tk.__init__(self, parent)
		self.parent = parent
		self.initialise()

	def initialise(self):
		self.grid()
		self.bind('<Key>', self.key)
		with open('image_list.txt') as f:
			self.file_list = f.readlines()

		self.label = Label(self, image=self.get_image(), width=100, height=100)
		self.label.pack()

	def get_image(self):
		image_name = self.file_list.pop(0).strip()
		self.filename = image_name
		self.image = PhotoImage(file=image_name)
		return self.image

	def key(self, event):
		key = event.char
		if ord(key) >= ord('A') and ord(key) <= ord('z') or key == ' ':
			print "Recognised as ", key
			self.label.configure(image=self.get_image())
			with open('lists/'+key+'.txt', 'a') as f:
				f.write(self.filename+"\n")

if __name__ == '__main__':
	app = classifier(None)
	app.title('Image classifier')
	app.mainloop()
