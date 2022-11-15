
class ArrayClass:
    
    def __init__(self):
        self.a = []
        self.size = 0

    def put(self, item):
        self.size += 1
        self.a.insert(0, item)
    
    def get(self):
        self.size -= 1
        return self.a[self.size]
    
    
if __name__ == "__main__":
    test = ArrayClass()
    test.put(1)
    print(test.get())
