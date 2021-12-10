import numpy as np
import run_model
import time
import Dense_CRF
import cv2 as cv
import sys,getopt

def extract_tree_mask(img,semantic_mask):
    blue = img[:,:,0] 
    ret, thresh = cv.threshold(blue, 150, 255, 0)
    thresh = cv.bitwise_not(thresh)
    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    im2 = np.zeros(blue.shape)
    cv.drawContours(im2, contours, -1, 255 , -1)
    return im2

def extract_pole_mask(img):
    red = img[:,:,0] 
    ret, thresh = cv.threshold(red, 220, 255, 0)
    thresh = cv.bitwise_not(thresh)
    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    im2 = np.zeros(red.shape)
    cv.drawContours(im2, contours, -1, 255 , -1)
    return im2

def extract_lane_mask(img, semantic_mask): 
    gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
    mask = (semantic_mask == 1)
    gray[semantic_mask !=1] = 0 


    mean =  np.sum(gray.flatten()) / np.sum(mask.flatten()) 
    std = np.sqrt(np.sum((gray[semantic_mask ==1]-mean)**2)/ np.sum(mask.flatten())) 



    gray = gray.astype('float32')
    gray[mask] = (gray[mask] -mean)/std*20 + 125
    gray = np.clip(gray,0,255)
    gray = gray.astype('uint8') 

     
    ret, thresh = cv.threshold(gray, 170, 256, 0) #before 170
    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE) 
    im2 = np.zeros(gray.shape) 
    cv.drawContours(im2, contours, -1, 255 , -1)
    return im2


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print('Usage: ./main img_path mask_path')
        exit(0)
    else:
        inputfile = sys.argv[1]
        outputfile = sys.argv[2]
    use_car = False
    try:
        opts, args = getopt.getopt(sys.argv[1:],"hi:o:v:",["ifile=","ofile=","use_car="])
    except getopt.GetoptError:
        print('test.py -i <inputfile> -o <outputfile> -v <whether use car>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('test.py -i <inputfile> -o <outputfile>')
            sys.exit()
        elif opt in ("-i", "--ifile"):
            inputfile = arg
        elif opt in ("-o", "--ofile"):
            outputfile = arg
        elif opt in ("-v", "--use_car"): 
            use_car = (arg == "use_car")


    gt_mask_name = ""

    img_name = inputfile 

    img = cv.imread(img_name)



    if gt_mask_name!="":
        gt_mask = cv.imread(gt_mask_name,cv.IMREAD_GRAYSCALE)
        pole_mask = extract_pole_mask(img) 

        road_mask = np.zeros(gt_mask.shape)
        for i in range(img.shape[0]):
            for j in range(img.shape[1]):
                if gt_mask[i,j] == 7:
                    road_mask[i,j] = 1 

        lane_mask = extract_lane_mask(img,road_mask) 


        merged_mask = np.zeros(gt_mask.shape) 
    
        for i in range(img.shape[0]):
            for j in range(img.shape[1]):  
                if gt_mask[i,j] == 17:
                    merged_mask[i,j] = 255
                elif gt_mask[i,j] == 7:
                        if lane_mask[i,j] >10:
                            merged_mask[i,j] = 255

    
    else : 
        (seg_map, seg_map_result) = run_model.run_model(img_name)

 


        CRF_mask = Dense_CRF.CRF_refine(seg_map,img_name,sxy=20, srgb=10, compat=5 , use_car = use_car)  #our case: 20 10 5
        CRF_mask_road = Dense_CRF.CRF_refine_road(seg_map,img_name,sxy=100, srgb=10, compat=5) 

        lane_mask = extract_lane_mask(img,CRF_mask_road) 



        merged_mask = np.zeros(lane_mask.shape) 
    
        for i in range(img.shape[0]):
            for j in range(img.shape[1]):  
                if CRF_mask[i,j] == 1:
                    merged_mask[i,j] = 255
                elif CRF_mask_road[i,j] == 1:
                        if lane_mask[i,j] >10:
                            merged_mask[i,j] = 255

 
    
 

             
    cv.imwrite(outputfile,merged_mask) 