# Python program to illustrate
# simple thresholding type on an image
	
# organizing imports
import cv2
import numpy as np

# path to input image is specified and
# image is loaded with imread command
image1 = cv2.imread('./map3.1.pgm')

# cv2.cvtColor is applied over the
# image input with applied parameters
# to convert the image in grayscale
img = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
img2 = img.copy()

def draw_grid(img, line_color=(0, 255, 0), thickness=1, type_=cv2.LINE_AA, pxstep=50):
    '''(ndarray, 3-tuple, int, int) -> void
    draw gridlines on img
    line_color:
        BGR representation of colour
    thickness:
        line thickness
    type:
        8, 4 or cv2.LINE_AA
    pxstep:
        grid line frequency in pixels
    '''
    x = pxstep
    y = pxstep
    while x < img.shape[1]:
        cv2.line(img, (x, 0), (x, img.shape[0]), color=line_color, lineType=type_, thickness=thickness)
        x += pxstep
    print()
    while y < img.shape[0]:
        cv2.line(img, (0, y), (img.shape[1], y), color=line_color, lineType=type_, thickness=thickness)
        y += pxstep

def draw_centers(image, color=(0,255,0), centres=[(0,0)], radius=1, thickness=-1):
    for _ in centres:
        image = cv2.circle(image, _, radius, color, thickness)
        
def draw_centers2(image, color=(0,255,0), pxstep=50, radius=1, thickness=-1, occupancy_grid=[]):
    # for _ in centres:
    x = pxstep//2
    y = pxstep//2
    while x < img.shape[1]:
        y = pxstep//2
        while y < img.shape[0]:
            if occupancy_grid is np.empty:
                image = cv2.circle(image, (x, y), radius, color, thickness)
            else:
                if occupancy_grid[y][x] == 1:
                    image = cv2.circle(image, (x, y), radius, color, thickness)
            y += pxstep
        x += pxstep
        
def kernel_sum(image,px=0,py=0, pxstep=5):
    roi = image[py : py+pxstep, px : px+pxstep]
    sum = np.sum(roi)
    # print(roi)
    # print(sum)
    return sum

def occupancy(th_img, th_val_per=70, pxstep=5):
    occupancy = np.zeros(th_img.shape)
    x = 0
    y = 0
    sum = 0
    max_sum = 255 * pxstep * pxstep
    if th_val_per > 100:
        th_val_per =100
    if th_val_per < 0:
        th_val_per =0
    th_sum = max_sum * (th_val_per/100)
    while x < img.shape[1]:
        y = 0
        while y < img.shape[0]:
            # image = cv2.circle(image, (x, y), radius, color, thickness)
            sum = kernel_sum(th_img,x,y,pxstep)
            if sum >= th_sum:
                occupancy[y][x]=1
                pass
            y += pxstep
        x += pxstep
    return occupancy
# applying different thresholding
# techniques on the input image
# all pixels value above 120 will
# be set to 255
ret, thresh1 = cv2.threshold(img, 120, 255, cv2.THRESH_BINARY)
# ret, thresh2 = cv2.threshold(img, 120, 255, cv2.THRESH_BINARY_INV)
# ret, thresh3 = cv2.threshold(img, 120, 255, cv2.THRESH_TRUNC)
# ret, thresh4 = cv2.threshold(img, 120, 255, cv2.THRESH_TOZERO)
# ret, thresh5 = cv2.threshold(img, 120, 255, cv2.THRESH_TOZERO_INV)
# draw_centers2(img, radius=1,pxstep=5); 

# the window showing output images
# with the corresponding thresholding
# techniques applied to the input images
# kernel_sum(thresh1,5,70,300)
# draw_grid(img, pxstep=5)
occp_grid = occupancy(thresh1, th_val_per=100)
# print(occp_grid)

# draw_centers2(img,pxstep=5 ,occupancy_grid=occp_grid,radius=50)

pxstep = 5
for _ in range(occp_grid.shape[0]):
    for __ in range(occp_grid.shape[1]):
        if occp_grid[_][__] == 1:
            # print(_,__)
            draw_centers(img,centres=[(__+pxstep//2,_+pxstep//2)],radius=1)

# cv2.imshow('img', img)
# cv2.imshow('Binary Threshold', thresh1)
# cv2.imshow('Binary Threshold Inverted', thresh2)
# cv2.imshow('Truncated Threshold', thresh3)
# cv2.imshow('Set to 0', thresh4)
# cv2.imshow('Set to 0 Inverted', thresh5)

# De-allocate any associated memory usage
if cv2.waitKey(0) & 0xff == 27:
	cv2.destroyAllWindows()


def nearest_point_on_graph(x,y,pxstep,occ_grid_shape0,off_x,off_y):
    new_x = np.round((x-off_x)/pxstep) * pxstep 
    new_y = np.round((y+off_y +occ_grid_shape0*pxstep)/pxstep) * pxstep
    print(new_x, new_y)
    new_x /= pxstep
    new_y /= pxstep
    print(new_x, new_y)
    pass
        
nearest_point_on_graph(1.5,0,1.2,10.5,-12,0)

##############################
img_res = 0.102 #in meters/pixel

grid_dist = 1.5 #in meters
grid_occp_per = 50 #in percentage

pix_dis = grid_dist / img_res
pix_dis = int(np.round(pix_dis))
#origin: [-55.4, 0.0, 0]
print("Pixel step: {}\nEffective grid width: {} meters\nOccp. grid thresholding: {}%".format(
    pix_dis,pix_dis*img_res,grid_occp_per))
# print(thresh1.shape)

print()

for y in range(occp_grid.shape[0]):
    for x in range(occp_grid.shape[1]):
        if occp_grid[y][x] ==  1:
            # print(x,y)
            # print([x * img_res, y * img_res])
            pass

ret, thresh = cv2.threshold(img2, 120, 255, cv2.THRESH_BINARY)
occp_grid = occupancy(thresh ,pxstep=pix_dis ,th_val_per=grid_occp_per)
for _ in range(occp_grid.shape[0]):
    for __ in range(occp_grid.shape[1]):
        if occp_grid[_][__] == 1:
            # print(_,__)
            draw_centers(img2,centres=[(__+pix_dis//2,_+pix_dis//2)],radius=1)
            

cv2.imshow('img', img2)
cv2.imshow('Binary Threshold', thresh)


# De-allocate any associated memory usage
if cv2.waitKey(0) & 0xff == 27:
	cv2.destroyAllWindows()
