import cv2
import cv2.aruco as aruco
import math
import cmath
import numpy as np
import random
import time
from collections import deque
import gym
import vision_arena
import pybullet as p
import pybullet_data
   # Function to return bot points (Aruco)

def bot_pos():
    while(1):
        img = env.camera_feed()
        aruco_dict=aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        parameters=aruco.DetectorParameters_create()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters)
        #print(corners)
        if(corners==[]):
            continue
        if(107 in ids):
            break
    xf=(corners[0][0][0][0]+corners[0][0][1][0])/2
    yf=(corners[0][0][0][1]+corners[0][0][1][1])/2
    xc=(corners[0][0][0][0]+corners[0][0][1][0]+corners[0][0][2][0]+corners[0][0][3][0])/4
    yc=(corners[0][0][0][1]+corners[0][0][1][1]+corners[0][0][2][1]+corners[0][0][3][1])/4
    xb=(corners[0][0][2][0]+corners[0][0][3][0])/2
    yb=(corners[0][0][2][1]+corners[0][0][3][1])/2
    return xb,yb,xf,yf,xc,yc

# Reading Arena

env = gym.make("vision_arena-v0")
time.sleep(1)
arena = env.camera_feed()

# Cropping arena

r = cv2.selectROI(arena)
print(r)
key = cv2.waitKey(0)
if(key ==ord('q')):
    cv2.destroyAllWindows()

# Calculating Dimension of arena

x1,y1,x2,y2=r
#width of 1 block
w=(x2-x1)/9
#height of one block
h=(y2-y1)/9
y1=y1+h/2
x1=x1+w/2
pos=np.zeros((9,9),dtype=int)
shape_dict={"SR":1,"TR":2,"CR":3,"SY":4,"TY":5,"CY":6}

#MASKING

hs=cv2.cvtColor(arena,cv2.COLOR_BGR2HSV)
mask1 = cv2.inRange(hs,np.array([0,70,50]),np.array([10,255,255]))
mask2=cv2.inRange(hs,np.array([170,20,50]),np.array([180,255,255]))
maskred=mask1+mask2
kernel = np.ones((5,5),np.uint8)
imred = cv2.erode(maskred,kernel,iterations = 1)
maskyel = cv2.inRange(hs,np.array([16,40,40]),np.array([35,255,255]))
imyel = cv2.erode(maskyel,kernel,iterations = 1)

# Create arena in readable format

contoursred,_=cv2.findContours(imred,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
contoursyel,_=cv2.findContours(imyel,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
for cnt in contoursred:
    area=cv2.contourArea(cnt)
    if(area<100):
        continue
    m=cv2.moments(cnt)    
    x=m['m10']/m['m00']
    y=m['m01']/m['m00']
    per=cv2.arcLength(cnt,True)
    ratio=(per**2)/area
    if(ratio>=15.5 and ratio<20):
        pos[min(round((y-y1)/h),8)][min(round((x-x1)/w),8)]=1
    elif(ratio>=20 and ratio<30):
        pos[min(round((y-y1)/h),8)][min(round((x-x1)/w),8)]=2
    elif(ratio>10 and ratio<15.5):
        pos[min(round((y-y1)/h),8)][min(round((x-x1)/w),8)]=3
    else:
        continue

for cnt in contoursyel:
    area=cv2.contourArea(cnt)
    if(area<100):
        continue
    m=cv2.moments(cnt)    
    x=m['m10']/m['m00']
    y=m['m01']/m['m00']
    per=cv2.arcLength(cnt,True)
    ratio=(per**2)/area
    if(ratio>=15.5 and ratio<20):
        pos[min(round((y-y1)/h),8)][min(round((x-x1)/w),8)]=4
    elif(ratio>=20 and ratio<30):
        pos[min(round((y-y1)/h),8)][min(round((x-x1)/w),8)]=5
    elif(ratio>10 and ratio<15.5):
        pos[min(round((y-y1)/h),8)][min(round((x-x1)/w),8)]=6
    else:
        continue  
pos[0][4]=pos[4][0]=pos[4][8]=pos[8][4]=0

print(pos)

# Function to update graph to add the edge to home

def update_graph(graph,start):
    print("updating")
    if(start==5):
        graph[23].insert(0,32)
        graph[32]=[]
    elif(start==37):
        graph[39].insert(0,40)
        graph[40]=[]
    elif(start==45):
        graph[43].insert(0,42)
        graph[42]=[]
    elif(start==77):
        graph[59].insert(0,50)
        graph[50]=[]
    else:
        pass
    return graph

def bfs(start,s,e,graph,br1,br2,br):
    parent={s:s}
    q=deque([])
    q.append(s)
    ans=0
    while ((ans==0) and (len(q)>0)):
        node=q.popleft()
        if(len(graph[node])==0):
            continue
        for i in set(graph[node]):
            if(i==parent[node]):
                continue
            if(br==True):
                if((i==br1) or (i==br2)):
                    continue
            parent[i]=node
            q.append(i)
            if (pos[((i-1)//9)][i-1-((i-1)//9)*9]==e):
                ans=i
                break
    path_st=[]
    path_st.append(i)
    if(ans==0):
        return path_st,graph,br,False
    while 1:
        if(parent[i]==i):
            break
        if(br==False):   
            if((path_st[-1]==br1) or (path_st[-1]==br2)):
                br=True
                graph=update_graph(graph,start)
        path_st.append(parent[i])
        i=parent[i]
    return path_st,graph,br,True

def cyc(s):
    if(s==5):
        home=32
        br1=6
        br2=24
    elif(s==37):
        home=40
        br1=28
        br2=30
    elif(s==45):
        home=42
        br1=54
        br2=52
    else:
        home=50
        br1=76
        br2=58
    return home,br1,br2

graph={}
for i in range(1,9):
    graph[i]=[]
    graph[i].append(i+1)
for i in range(1,9):
    graph[i*9]=[]
    graph[i*9].append((i+1)*9)
for i in range(1,9):
    graph[81-i+1]=[]
    graph[81-i+1].append(81-i)
for i in range(1,9):
    graph[(9-i)*9+1]=[]
    graph[(9-i)*9+1].append((9-i-1)*9+1)
    
    
for i in range(1,5):
    graph[20+i]=[]
    graph[20+i].append(20+i+1)
for i in range(1,5):
    graph[16+i*9]=[]
    graph[16+i*9].append(16+(i+1)*9)
for i in range(1,5):
    graph[61-i+1]=[]
    graph[61-i+1].append(61-i)
for i in range(1,5):
    graph[21+(5-i)*9]=[]
    graph[21+(5-i)*9].append(21+(5-i-1)*9)    
    
graph[5].append(14)
graph[14]=[5,23]
graph[23].append(14)
graph[37].append(38)
graph[39].append(38)
graph[38]=[37,39]
graph[43].append(44)
graph[45].append(44)
graph[44]=[43,45]
graph[77].append(68)
graph[59].append(68)
graph[68]=[77,59]

# Function to move bot with given points

def move_bot(x,y,current):
    while (1):
        xb,yb,xf,yf,xc,yc=bot_pos()
        path_vec=cmath.phase(complex(x-xb,yb-y))
        bot_vec=cmath.phase(complex(xf-xc,yc-yf))
        #print(np.degrees(path_vec-bot_vec))
        if((abs(x-xc)+abs(y-yc))<=1.5):
            #print("break above")
            #print((abs(x-xc)+abs(y-yc)))
            break
        #lprint(np.degrees(path_vec-bot_vec))
        if((179.5>=np.degrees(path_vec-bot_vec)>=0.5) or (-180.5>=np.degrees(path_vec-bot_vec)>=-359.5)):
            #print("left")
            #print(np.degrees(path_vec-bot_vec))
            start=time.perf_counter()
            stop=time.perf_counter()
            while(stop-start)<0.01:
                p.stepSimulation()
                env.move_husky(-1.35,2,-1.35,2)
                stop=time.perf_counter()

        elif(abs(np.degrees(path_vec-bot_vec))<0.5):
            if((abs(x-xc)+abs(y-yc))>1.5):
                #print("forward")
                #print((abs(x-xc)+abs(y-yc)))
                start=time.perf_counter()
                stop=time.perf_counter()
                while(stop-start)<0.01:
                    p.stepSimulation()
                    env.move_husky(2.5,2.5,2.5,2.5)
                    stop=time.perf_counter()
            else:
                #print("break forward")
                #print((abs(x-xc)+abs(y-yc)))
                break
        else:
            #print("Right")
            #print(np.degrees(path_vec-bot_vec))
            start=time.perf_counter()
            stop=time.perf_counter()
            while(stop-start)<0.01:
                p.stepSimulation()
                env.move_husky(2,-1.35,2,-1.35)
                stop=time.perf_counter()
    #print("presss one block")
    #innnnn=input()

#Final Moving func

xb,yb,xf,yf,xc,yc=bot_pos()
start=(min(round((yc-y1)/h),8)*9)+min(round((xc-x1)/w),8)+1
print("Starting point",start)
a=start
home,br1,br2=cyc(start)
br=False
k=1
while(k):
    print("moving dice")
    #time.sleep(1)
    b=env.roll_dice()
    #b=random.randrange(1,7)
    print("got shape",b)
    #print("press any key")
    #garbage=input()
    b=shape_dict[b]
    path_st,graph,br,move=bfs(start,a,b,graph,br1,br2,br)
    if(move==False):
        print("Not allowed to move , rolling again")
        continue
    print("got path")
    print(path_st[::-1])
    current=path_st.pop()
    
    while (len(path_st)>0):
        point=path_st.pop()
        x=x1+((point-1)%9)*w
        y=y1+((point-1)//9)*h
        #print(point,"co-ordinate","(",x,",",y,")")
        move_bot(x,y,current)
        current=point
        if(point==home):
            move_bot(x1+4*w,y1+4*h,current)
            print("yayyyy")
            k=0
            break
    #print("press key to roll dice and hit enter")
    #garbage=input()
    a=point                
