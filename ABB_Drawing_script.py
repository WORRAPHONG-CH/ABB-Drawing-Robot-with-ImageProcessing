import re
import cv2,time,socket
import numpy as np 
import math as m
from tsp_solver.greedy import solve_tsp

def canVal_1(val1):
    pass
def canVal_2(val2):
    pass
cv2.namedWindow("Canny Value",cv2.WINDOW_FREERATIO)
cv2.createTrackbar("th1","Canny Value",128,255,canVal_1)
cv2.createTrackbar("th2","Canny Value",200,255,canVal_2)

# Calculate distance between points
def distance(x1,y1,x2,y2):
    dist = float(m.sqrt((x1-x2)**2 + (y1-y2)**2))
    return dist

# Store position and draw images
def draw_image(point,x,y,path,factor):
    #x,y is new size of the image
    #point is edges points 
    print("----> Drawing Process <----")
    cv2.namedWindow("Drawing img[Optimize path]",cv2.WINDOW_FREERATIO)
    array_x = []
    array_y = []
    array_z = []
    #create background image for drawing 
    img_bg = np.zeros((x*factor,y*factor,3),np.uint8)
    
    for pos in range(1,(len(path))-1):
        #path help to sort point for drawing 
        x1,y1 = point[path[pos]]
        x2,y2 = point[path[pos+1]]
        dist_path = distance(x1,y1,x2,y2)
        print("No:",pos)
        print("point1:",path[pos])
        print("point2:",path[pos+1])
        print("x1:",x1)
        print("y1:",y1)
        print("x2:",x2)
        print("y2:",y2)
        print("---------------")
        
        
        if dist_path <= 2: 
            #array for drawing 
            array_x.append(str(x1))
            array_y.append(str(y1))
            array_z.append(str(0))
            
            array_x.append(str(x2))
            array_y.append(str(y2))
            array_z.append(str(0))
            
            time.sleep(0.01)
            cv2.line(img_bg,(x1,y1),(x2,y2),(0,255,255),1)
            cv2.imshow("Drawing img[Optimize path]",img_bg)
            cv2.waitKey(1)
        
            
        else:
            #array for lifting the pen 
            array_x.append(str(x1))
            array_y.append(str(y1))
            array_z.append(str(-5))
            
            array_x.append(str(x2))
            array_y.append(str(y2))
            array_z.append(str(-5))
            pass
        
    print("**** Finished Drawing ****")   
    return array_x,array_y,array_z     
   

# Post-process image
def image_processing(img,factor=1,w=290,h=210):
    print("---->>image processing<<----")
    cv2.namedWindow("image",cv2.WINDOW_FREERATIO)
    
    img = cv2.imread(img)
    x,y,ch = img.shape
    
    max_value = max(x,y)
    #print("max(x,y):",max_value)
    
    desired_size = [w,h]
    desired = [desired_size[0]*(y/max_value),
               desired_size[1]*(x/max_value), #65% of 1920(350) 
                3]
    print("desired size:",desired)
    
    scale = [desired[0]/y,desired[1]/x]
    
    img_rgb = cv2.resize(img,None,fx=scale[0],fy=scale[1],interpolation=cv2.INTER_CUBIC)
    
    gray = cv2.cvtColor(img_rgb,cv2.COLOR_BGR2GRAY)
    
    while(1):
        
        th1 = cv2.getTrackbarPos("th1","Canny Value")
        th2 = cv2.getTrackbarPos("th2","Canny Value")
        #edges = cv2.Canny(gray,0,255,apertureSize = 3)
        edges = cv2.Canny(gray,th1,th2)#,apertureSize = 3
        
        button1 = cv2.waitKey(1)
        cv2.imshow("image",img_rgb)
        cv2.imshow("Canny",edges)
       
        if button1 & 0xFF == ord("e"):
            break
        
        elif button1 & 0xFF == ord("s"):
            img_name = "output/CannyImg.png"
            cv2.imwrite(img_name,edges)
            print("**** Save Image ****")
            
            # Store edge image
            edges_img = cv2.imread("CannyImg.png")
            edges_list = edges_img.tolist()
            print("point(edges):",len(edges_list))
                
            # Downscale image 
            new_x, new_y, ch =  img_rgb.shape
            print("original size:",img.shape)
            print("new size:",img_rgb.shape)
                
            #Store only white(255) point or edges
            point_x = []
            point_y = []
                
            print("----> Store Only White point <----")
            for rows in range(1,new_y):
                for cols in range(1,new_x):
                    if edges[cols][rows] == 255:
                        point_x.append(rows)
                        point_y.append(cols)
                        #print(".")
                            
            points_edges = list(zip(point_x, point_y))
            print("No. of Position Edge(x,y):",len(points_edges))
            #print(points_edges)
                
            r = [[0 for x in range(len(point_x))] for y in range(len(point_x))]
            print("size r: ",len(r))
                
            #calculate distance each point(edges)
            for p1 in range(1,len(point_x)):     #row(y)
                for p2 in range(1,len(point_x)): #col(x)
                    x1,y1 = points_edges[p1]
                    x2,y2 = points_edges[p2]
                    r[p1][p2] = distance(x1,y1,x2,y2)
                    #print(".")
            print("**** Distance calculated ***")
                
            #TSP -library use to optimizes the path that will be drawing
            #greedy method will choose the optimize path to reduce time to find target(fastest way)
            print("---->> Solving TSP <<----")
            path = solve_tsp(r)
            print("**** TSP Done ****")
            
            
            # Convert a list to array to multiply with factor
            #factor = 1 -> normal | < 1 -> reduce(0.5) | > 1 -> increase(1.5)
            np_pointEdges = np.array(points_edges)
            np_pointEdges = np_pointEdges*factor
            points_edges = np_pointEdges.tolist() #then convert array back to list
            print("#### Image Processed ####")
            return points_edges,new_x,new_y,path,factor

# Write script file to use in RobotStudio
def script_RAPID(module_name,tool_name,array_x,array_y,array_z):
    print("----> Creating Script <----")
    count = 0 
    try:
        #mode r+ means open a file for reading updating 
        file = open(module_name+("{}.mod").format(count),mode="r+")
        
    except(FileNotFoundError):
        print("#### File not found ####")
        print("----> Creating Script file <----")
        #mode w means opens a file for writing. Create new file if it does not exists.
        count +=1
        
        file = open(module_name+("{}.mod").format(count),mode="w")
        #file = open(module_name+("{}.txt").format(count),mode="w")
        file = open(module_name+("{}.mod").format(count),mode="r+")
        #file = open(module_name+("{}.txt").format(count),mode="r+")
        
    #open(module_name+("{}.prg").format(count),mode="w").close()
    #open(module_name+("{}.txt").format(count),mode="w").close()
    open(module_name+("output/{}.mod").format(count),mode="w").close()
    
    #HEADER 
    file.write("\nMODULE "+module_name+str(count)+"\n")
    file.write("\n\tPERS tooldata "+tool_name+":=[TRUE,[[0,0,139],[1,0,0,0]],[0.5,[0,0,20],[1,0,0,0],0,0,0]];")
    file.write("\n\tCONST robtarget origin :=[[0,0,0],[1,0,0,0],[0,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];")
    file.write("\n\tCONST robtarget home_1:=[[120,148.105008887,-250.933670218],[0.706940789,-0.015320621,0.015320621,0.706940789],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];")
    file.write("\n\tTASK PERS wobjdata paper_obj:=[FALSE,TRUE,\"\",[[1355,120,401],[0,-0.707106781,0.707106781,0]],[[0,0,0],[1,0,0,0]]];")
    file.write("\n\tVAR num array_x{"+str(len(array_x))+"}:= ["+(','.join(array_x))+"];")
    file.write("\n\tVAR num array_y{"+str(len(array_y))+"}:= ["+(','.join(array_y))+"];")
    file.write("\n\tVAR num array_z{"+str(len(array_z))+"}:= ["+(','.join(array_z))+"];")
    
    #Program 
    file.write("\n\tPROC main()")
    file.write("\n\t\tMoveL home_1,v800,z100,pen\WObj:=paper_obj;")
    file.write("\n\t\tFOR i FROM 1 TO Dim(array_x,1) DO")
    file.write("\n\t\t\tMoveL Offs (origin,array_x{i},array_y{i},array_z{i}),v200,fine,pen\WObj:=paper_obj;")
    file.write("\n\t\tENDFOR")
    file.write("\n\t\tMoveL home_1,v800,z100,pen\WObj:=paper_obj;")
    file.write("\n\tENDPROC")
    file.write("\nENDMODULE")
    file.close()
    print("**** Script File Created ****")
    
####------------------------------------------------  MAIN PROGRAM ------------------------------------------------####


img_path = "imgs/ANV4y3.jpg"
#factor = 1 -> normal | < 1 -> reduce(0.5) | > 1 -> increase(1.5)
point,x,y,path,factor = image_processing(img_path,factor=1,w=290,h=210)

array_x,array_y,array_z = draw_image(point,x,y,path,factor)

script_RAPID("ANV4y","Pen",array_x,array_y,array_z)

button = cv2.waitKey(0)
if button & 0xFF == 27:
    cv2.destroyAllWindows()
    