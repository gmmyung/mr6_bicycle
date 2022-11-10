import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import serial

import signal
import threading
from queue import Queue

line = [] #라인 단위로 데이터 가져올 리스트 변수

port = 'COM5' # 시리얼 포트
baud = 38400 # 시리얼 보드레이트(통신속도)

exitThread = False   # 쓰레드 종료용 변수

dataQueue = Queue()


#쓰레드 종료용 시그널 함수
def handler(signum, frame):
    exitThread = True


#데이터 처리할 함수
def parsing_data(data):
    # 리스트 구조로 들어 왔기 때문에
    # 작업하기 편하게 스트링으로 합침
    tmp = ''.join(data)

    return tmp

#본 쓰레드
def readThread(ser):
    global line
    global exitThread

    # 쓰레드 종료될때까지 계속 돌림
    while not exitThread:
        #데이터가 있있다면
        for c in ser.read():
            #line 변수에 차곡차곡 추가하여 넣는다.
            line.append(chr(c))

            if c == 10: #라인의 끝을 만나면..
                #데이터 처리 함수로 호출
                d = parsing_data(line)
                #Queue.put(d)
                print(d)

                #line 변수 초기화
                del line[:]              


# 스코프 클래스 정의
class Scope(object):

    # 초기 설정
    def __init__(self, ax, fn, xmax=10,ymax =10, xstart=0, ystart=0, title='Title', xlabel='X value', ylabel='Y value'):
        
        self.xmax = xmax #x축 길이
        self.xstart = xstart #x축 시작점
        self.ymax = ymax #y축 길이
        self.ystart = ystart #y축 시작점

        # 그래프 설정
        self.ax = ax 
        self.ax.set_xlim((self.xstart,self.xmax))
        self.ax.set_ylim((self.ystart,self.ymax))
        self.ax.set_title(title)
        self.ax.set_xlabel(xlabel)
        self.ax.set_ylabel(ylabel)

        self.x = [0] # x축 정보 
        self.y = [0] # y축 정보
        self.value = 0 # 축 값
        self.fn = fn
        self.line, = ax.plot([],[])

        self.ti = time.time() #현재시각
        print("초기화 완료")

    # 그래프 설정
    def update(self, i):
        # 시간차
        tempo = time.time()-self.ti
        self.ti = time.time() #시간 업데이트
        
        # 값 넣기
        self.value = self.fn()# y값 함수 불러오기
        self.y.append(self.value) #y값 넣기
        self.x.append(tempo + self.x[-1]) #x값 넣기
        self.line.set_data(self.x,self.y)

        # 화면에 나타낼 x축 범위 업데이트
        if self.x[-1] >= self.xstart + self.xmax :
            #전체 x값중 반을 화면 옆으로 밀기
            self.xstart = self.xstart + self.xmax/2
            self.ax.set_xlim(self.xstart,self.xstart + self.xmax)

            self.ax.figure.canvas.draw()

        return (self.line, )


fig, ax = plt.subplots()
ax.grid(True)

#종료 시그널 등록
signal.signal(signal.SIGINT, handler)

#시리얼 열기
ser = serial.Serial(port, baud, timeout=0)

#시리얼 읽을 쓰레드 생성
thread = threading.Thread(target=readThread, args=(ser,))

#시작!
thread.start()


# y축에 표현할 값을 반환해야하고 scope 객체 선언 전 선언해야함.
def insert():
    value = dataQueue.get()
    return value 

# 객체 생성
scope = Scope(ax,insert, ystart = 0, ymax = 10)
    
# update 매소드 호출
ani = animation.FuncAnimation(fig, scope.update,interval=10,blit=True)
plt.show()