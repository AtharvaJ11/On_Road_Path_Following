#!/usr/bin/env python
# coding: utf-8




import numpy as np
import pandas as pd
# import matplotlib.pyplot as plt
from shapely.geometry import LineString
from shapely.geometry import Point
from pprint import pprint
import json
import rospy
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Point as Point_ROS, Quaternion
from std_msgs.msg import Header
from nav_msgs.msg import Odometry, Path





dict1 = json.load(open('paths.json', 'r'))

dict = json.load(open('paths.json', 'r'))




for i in dict:
    print(len(dict[i]))





import numpy as np

# fig = plt.figure(1, dpi=200)

# ax = fig.add_subplot(111)

# for i in dict:
#     data = np.array(dict[i])
#     plt.plot(data[:, 0], data[:, 1], '-o', linewidth=.75, markersize=2)
    
# plt.show()




count=0
for i in dict:
    for j in range(len(dict[i])):
        
        arr = np.array([dict[i][j]])
#         np.append(arr, dict[i][j])
        arrx = [i]
        arry = [j]
        for k in dict:
            for l in range(len(dict[k])):
                if i!=k or i==k:
                    if Point(dict[k][l]).distance(Point(dict[i][j]))<2.3 and Point(dict[k][l]).distance(Point(dict[i][j]))>0:
                        np.append(arr, dict[k][l])
                        arrx.append(k)
                        arry.append(l)
                        count+=1
                        
        t = np.mean(arr, axis=0)

       
        
        for c in range(len(arrx)):
            m = int(arry[c])
            n = (arrx[c])
            dict[n][m] = t
            
            
    
count





import numpy as np

# fig = plt.figure(1, dpi=200)

# ax = fig.add_subplot(111)

# for i in dict:
#     data = np.array(dict[i])
#     plt.plot(data[:, 0], data[:, 1], '-o', linewidth=.75, markersize=2)
    
# plt.show()





dict





count=0
for i in dict:
    for j in range(len(dict[i])):
        for k in dict:
            for l in range(len(dict[k])):
                if i!=k:
                    if Point(dict[k][l]).distance(Point(dict[i][j]))<2.3 and Point(dict[k][l]).distance(Point(dict[i][j]))>0:
                        s = Point(dict[k][l]).distance(Point(dict[i][j]))
                        print(s)
                        count+=1
#                         arr1 = np.array(dict[k][l])
#                         arr2 = np.array(dict[i][j])
#                         avg = (arr1 + arr2) / 2
#                         dict[i][j] = avg
#                         dict[k][l] = avg
                        

                        
#                        s = Point(j).distance(Point(l))
#                        print(s)
#                         count+=1;
                    
print(count)                





dict






    
    
def addindex(arr, p):

    for g in range(len(dict[p])):            
        if g==len(dict[p])-1:
            if arr[0]==dict[p][g][0] and arr[1]==dict[p][g][1]:
                continue
            
        elif arr[0]==dict[p][g][0] and arr[1]==dict[p][g][1]:
            continue
        else:
            if ((arr[0]-dict[p][g][0])*(arr[0]-dict[p][g+1][0])<0) and ((arr[1]-dict[p][g][1])*(arr[1]-dict[p][g+1][1])<0):
                dict[p].insert(g+1, arr)
                
    return

def returnindex(arr, p):
    
    listt = set()
    for g in range(len(dict[p])):            
        if arr[0]==dict[p][g][0] and arr[1]==dict[p][g][1]:
            listt.add(g)
        
                
    listtt = list(listt)
    return listtt
            
    
    
sett = set()

for i in dict:
    sett.add(i)
    for j in dict:
        if (j not in sett):
            a = LineString(dict[i])
            b = LineString(dict[j])
            c = a.intersection(b)


        
            if c.type=="LineString":
                x, y = c.xy
                for a in range(len(x)):
                    arr = [0, 0]
                    arr[0] = x[a]
                    arr[1] = y[a]
                    list1 = addindex(arr, i)
                    list2 = addindex(arr, j )

#                     for k in list1:
#                         node_dict[i].append(k)
#                         for l in list2:
#                             node_dict[j].append(l)
#                             string1 = "dict[i]"
#                             string1.append("k")
#                             string1.append(dict[j])
#                             string1.append("l")
                            # for graph_dict
                
            elif c.type=="Point":
                x, y = c.xy
                arr=[0, 0]
                arr[0] = x[0]
                arr[1] = y[0]
                list1 = addindex(arr, i)
                list2 = addindex(arr, j)

#                 for k in list1:
#                     node_dict[i].append(k)
#                     for l in list2:
#                         node_dict[j].append(l)
#                         string1 = "dict[i]"
#                         string1.append("k")
#                         string1.append(dict[j])
#                         string1.append("l")
                
                
            else:
                for ob in c:
                    x, y = ob.xy
                    for a in range(len(x)):
                        arr = [0, 0]
                        arr[0] = x[a]
                        arr[1] = y[a]
                        list1 = addindex(arr, i)
                        list2 = addindex(arr, j)

#                         for k in list1:
#                             node_dict[i].append(k)
#                             for l in list2:
#                                 node_dict[j].append(l)
#                                 string1 = "dict[i]"
#                                 string1.append("k")
#                                 string1.append(dict[j])
#                                 string1.append("l")
                                # for graph_dict



               
dict


# np.array[len(dict)]
 


# graph_dict

# node_dict

    


# print(1)



node_dict  = {}
# print(node_dict["p1"])
for i in dict:
    node_dict[i] = []
    node_dict[i].append(0)
    node_dict[i].append(len(dict[i])-1)




sett = set()

for i in dict:
    sett.add(i)
    for j in dict:
        if (j not in sett):
            a = LineString(dict[i])
            b = LineString(dict[j])
            c = a.intersection(b)


        
            if c.type=="LineString":
                x, y = c.xy
                for a in range(len(x)):
                    print(a)
                    arr = [0, 0]
                    arr[0] = x[a]
                    arr[1] = y[a]
                    list1 = returnindex(arr, i)
                    list2 = returnindex(arr, j )

                    for k in list1:
                        node_dict[i].append(k)
                        for l in list2:
                            node_dict[j].append(l)
                            
#                             string1 = "dict[i]"
#                             string1.append("k")
#                             string1.append(dict[j])
#                             string1.append("l")
                            # for graph_dict
                
            elif c.type=="Point":
                x, y = c.xy
                arr=[0, 0]
                arr[0] = x[0]
                arr[1] = y[0]
                list1 = returnindex(arr, i)
                list2 = returnindex(arr, j)

                for k in list1:
                    node_dict[i].append(k)
                    for l in list2:
                        node_dict[j].append(l)
#                         string1 = "dict[i]"
#                         string1.append("k")
#                         string1.append(dict[j])
#                         string1.append("l")
                
                
            else:
                for ob in c:
                    x, y = ob.xy
                    for a in range(len(x)):
                        arr = [0, 0]
                        arr[0] = x[a]
                        arr[1] = y[a]
                        list1 = returnindex(arr, i)
                        list2 = returnindex(arr, j)

                        for k in list1:
                            node_dict[i].append(k)
                            for l in list2:
                                node_dict[j].append(l)
#                                 string1 = "dict[i]"
#                                 string1.append("k")
#                                 string1.append(dict[j])
#                                 string1.append("l")
                                # for graph_dict

    
    
for i in node_dict:
    print(i)
    print(node_dict[i])





node_set = set()

node_path={}



for i in node_dict:
    for j in node_dict[i]:
        s = i+"_"+str(j)
        node_set.add(s)
        node_path.update({s : [0, "h"]})
# for j in dict:
    
#     a = j +"_" + "0"
#     b = len(dict[j])
#     c = j+"_"+str(b)
#     node_set.add(a)
#     node_path.update({a : [0, "h"]})
#     node_set.add(c)
#     node_path.update({c : [0, "h"]})
    
    
    
node_path





def initialize(node_path):
    for i in node_path:
        node_path[i][0]=1000000000000
        node_path[i][1]="."
    return





initialize(node_path)
node_path





node_graph={}
for i in node_set:
    node_graph.update({i : []})
    
node_graph



node_graph={}
for i in node_set:
    node_graph.update({i : []})
    
node_graph

sett = set()

for i in dict:
    sett.add(i)
    for j in dict:
        if (j not in sett):
            a = LineString(dict[i])
            b = LineString(dict[j])
            c = a.intersection(b)


        
            if c.type=="LineString":
                x, y = c.xy
                for a in range(len(x)):
                    print(a)
                    arr = [0, 0]
                    arr[0] = x[a]
                    arr[1] = y[a]
                    list1 = returnindex(arr, i)
                    list2 = returnindex(arr, j )

                    for k in list1:
#                         node_dict[i].append(k)
                        for l in list2:
#                             node_dict[j].append(l)
                            
                            a = i+"_"+str(k)
                            b = j+"_"+str(l)
                            
                            ar = [b, 0]
                            
                            node_graph[a].append(ar)
                            arr=[a, 0]
                            node_graph[b].append(arr)
                            
#                             string1 = "dict[i]"
#                             string1.append("k")
#                             string1.append(dict[j])
#                             string1.append("l")
                            # for graph_dict
                
            elif c.type=="Point":
                x, y = c.xy
                arr=[0, 0]
                arr[0] = x[0]
                arr[1] = y[0]
                list1 = returnindex(arr, i)
                list2 = returnindex(arr, j)

                for k in list1:
#                     node_dict[i].append(k)
                    for l in list2:
#                         node_dict[j].append(l)
                        
                        a = i+"_"+str(k)  
                        b = j+"_"+str(l)    
                        ar = [b, 0]
                            
                        node_graph[a].append(ar)
                        arr=[a, 0]
                        node_graph[b].append(arr)
#                         string1 = "dict[i]"
#                         string1.append("k")
#                         string1.append(dict[j])
#                         string1.append("l")
                
                
            else:
                for ob in c:
                    x, y = ob.xy
                    for a in range(len(x)):
                        arr = [0, 0]
                        arr[0] = x[a]
                        arr[1] = y[a]
                        list1 = returnindex(arr, i)
                        list2 = returnindex(arr, j)

                        for k in list1:
#                             node_dict[i].append(k)
                            for l in list2:
#                                 node_dict[j].append(l)
                                a = i+"_"+str(k)
                                b = j+"_"+str(l)    
                                ar = [b, 0]
                                node_graph[a].append(ar)
                                arr=[a, 0]
                                node_graph[b].append(arr)
                                
                                
#                                 string1 = "dict[i]"
#                                 string1.append("k")
#                                 string1.append(dict[j])
#                                 string1.append("l")
                                # for graph_dict

    
    
node_graph



node_sorted={}
for i in node_dict:
    lis = node_dict[i]
    lis.sort()
    node_sorted.update({i: lis})
    
for j in node_sorted:
    print(j)
    print(node_sorted[j])



for i in node_graph:
    
    a=""
    b=""
    c=0
    d=0
    
    for j in range(len(i)):
        if i[j]=="_":
            a = i[0:j]
            b = i[j+1:len(i)]
            c = int(b)
            
    length = len(node_sorted[a])
            
    for k in range(len(node_sorted[a])):
        if node_sorted[a][k]==c:
            d = k
    for kk in range(len(node_sorted[a])):
        if kk+d>=length:
            break
        else:
            if node_sorted[a][d+kk]>c:
                num = node_sorted[a][d+kk]
                f = LineString(dict[a][c:num+1])
                lenn = f.length
                B = a+"_"+str(num)
                ar = [B, lenn]
                node_graph[i].append(ar)
                break
    for K in range(length):
        if d-K<0:
            break
        else:
            if node_sorted[a][d-K]<c:
                numm = node_sorted[a][d-K]
                ff = LineString(dict[a][numm:c+1])
                lenn = ff.length
                B = a+"_"+str(numm)
                arr = [B, lenn]
                node_graph[i].append(arr)
                break
                
                
        
node_graph




from queue import Queue

def dijkstra(src):
    
    node_path[src][0]=0
    node_path[src][1]=""
    queue = Queue(maxsize = 100000)
    visited_node = set()
    queue.put(src)
    
    while not queue.empty():
        i = queue.get()
        if i not in visited_node:
            visited_node.add(i)
            for j in node_graph[i]:
                queue.put(j[0])
                if(node_path[i][0]+j[1]<=node_path[j[0]][0]):
                    node_path[j[0]][0] = node_path[i][0]+j[1]
                    node_path[j[0]][1] = node_path[i][1]+j[0]
    return node_path


# print(dijkstra("p8_7")["p6_10"])



# print(pathh)

def clbk(msg):
    global dest
    pt = [msg.position.x, msg.position.y]
    if pt==dest:
        return
    else:
        dest = pt

def odom_cb(msg):
    global once
    if once==0:
        global odom,x_odom,y_odom
        odom = msg

        x_odom = 1
        y_odom = 1

        x_odom = odom.pose.pose.position.x
        y_odom = odom.pose.pose.position.y



def array2path(pos_array):
    global path_seq
    path = Path()
    # path.header.frame_id = 'path header #777'
    path.header.frame_id = 'map'
    path.header.seq = path_seq
    pose_stamped_array = []
    i = 0
    for pos in pos_array:
        i+=1
        pose_stamped = PoseStamped() 
        pose_stamped.header.seq = i
        # pose_stamped.header.frame_id = 'poses header #666'
        pose_stamped.header.frame_id = 'map'
        pose_stamped.pose.position.x = pos[0]
        pose_stamped.pose.position.y = pos[1]
        pose_stamped_array.append(pose_stamped)
    path.poses = pose_stamped_array
    # print(path)
    return path

# pth = array2path(pathh)


# once = 0

if __name__ == "__main__":
    global once

    once=0
    ###VERY SIMPLE TO UNDERSTAND CONVERSION TO ROSPY FROM PYTHON
    rospy.init_node("GlobalPlanner")
    dest = None
    odom = None
    x,y = 1, 1
    
    temp_dest = None
    path_seq = 0
    x_odom, y_odom =  None, None

    




    path_publisher = rospy.Publisher("/path", Path, queue_size=10)
    destination_sub = rospy.Subscriber("/destination", Pose, clbk)
    odom_sub = rospy.Subscriber("/uwb_odom", Odometry, odom_cb)




    distancee = 10000

    soco = [1000, 1000]
    soco_s =""


    # print(len(dict["p1"]))




    for i in dict:
        for j in range(len(dict[i])):
            print(soco)
            if Point(soco).distance(Point([x_odom, y_odom]))<distancee:

                distancee=Point(soco).distance(Point([x_odom, y_odom])) 

                soco = dict[i][j]
                soco_s = i + str(j)


    i = 0
    r = rospy.Rate(1) #1hz
    while rospy.is_shutdown is not True:
                            
        if (temp_dest != dest) and (x_odom is not None):
            
            once=1
            
            path_seq+=1
            temp_dest = dest
            x,y = dest

            s=""
            s_temp = None

            # for i in dict:
            #     for j in range(len(dict[i])):
            #         if x==i[j][0] and y==i[j][1]:
            #             s_temp = s + i + str(j)

            distanceee=1000000

            s_tempp = [1000000, 1000000]

            for i in dict:
                for j in range(len(dict[i])):
                    if Point(s_tempp).distance(Point([x_odom, y_odom]))<distanceee:
                        distanceee=Point(s_tempp).distance(Point([x_odom, y_odom]))
                        
                        s_tempp = dict[i][j]
                        s_temp = s + i + str(j)


            x_path = dijkstra(soco_s)

            y_path = x_path[s_temp]

            z = y_path[1]

            destination = soco_s + z
            countt=0

            final = []

            for i in range(len(destination)):
                
                if destination[i]=='p':
                    nodee =destination[i]
                    for j in range(i+1, len(destination)):
                        if destination[j]=='p' or j == len(destination)-1:
                            if j == len(destination) -1:
                                nodee = nodee+destination[j]
                                final.append(nodee)
                                break
                            else:


                                final.append(nodee)

                                break
                        else:
                            nodee = nodee+destination[j]

            pathh = []

            for i in final:
                indd = 0
                for j in range(len(i)):
                    if(i[j]=='_'):
                        indd=j
                
                s1 = i[0:indd]
                s2 = int(i[indd+1:  len(i)])

                pathh.append(dict[s1][s2])
            i+=1

        
            rospy.loginfo("Path published [%s]",pathh)



        ##do some stuff
            pth = array2path(pathh)


        ##then publish your desired object in desired publisher
        once=0
        path_publisher.publish(pth)
        r.sleep()
        

        
    ##This keeps your code running









