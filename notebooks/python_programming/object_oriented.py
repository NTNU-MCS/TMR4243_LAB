import math

class Animal:
    name = None
    def __init__(self):
        """This is the constructor"""
        print("Animal constructor")

    def eat(self, food):
        print(f"{self.name} ate {food}")

    def set_name(self, name):
        self.name = name

    @staticmethod
    def whatis():
        print("just an animal, out of the scope of this course")

class Cat(Animal):
    def __init__(self):
        super().__init__()
        print("Cat constructor")
        self.attitude = None

a_cat = Cat()

Animal.set_name(a_cat, "furry")
a_cat.set_name("furry")

a_cat.eat("banana")