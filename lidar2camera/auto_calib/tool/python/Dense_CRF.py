import numpy as np
import cv2 as cv
import pydensecrf.densecrf as dcrf
from pydensecrf.utils import unary_from_softmax

def CRF_refine(semantic_mask,image_name, sxy, srgb, compat, use_car):

 
    final_mask = np.zeros([semantic_mask.shape[0],semantic_mask.shape[1],2])

    for i in range(semantic_mask.shape[0]):
        for j in range(semantic_mask.shape[1]): 
            # if semantic_mask[i,j,0]+semantic_mask[i,j,2]+semantic_mask[i,j,10]>0.5:
            #     final_mask[i,j,0] = 0.51
            # else:
            #     final_mask[i,j,0] = 0.49
            final_mask[i,j,1] =  semantic_mask[i,j,5]
            if use_car:
                 final_mask[i,j,1] =  final_mask[i,j,1] + semantic_mask[i,j,13]+ semantic_mask[i,j,11]
            final_mask[i,j,0] = 1-final_mask[i,j,1]
            
    
    final_mask = np.moveaxis(final_mask,-1,0)  

    semantic_mask = np.moveaxis(semantic_mask,-1,0)

    

    U = unary_from_softmax(final_mask) 
    img = cv.imread(image_name)  

    print (semantic_mask.shape)
    print (img.shape)

    img = img.copy(order = 'C')
    U = U.copy(order='C')
    U = U.astype(np.float32) 

    sxy_list_1 = np.arange(10,200,50)
    compat_list_1 = np.arange(1,20,5)
    sxy_list = np.arange(10,200,50)
    srgb_list = np.arange(3,14,3)
    compat_list = np.arange(1,20,5)

    # for sxy_1 in sxy_list_1:
    #     for compat_1 in compat_list_1:
    #         for sxy in sxy_list:
    #             for srgb in srgb_list:
    #                 for compat in compat_list:      
    d = dcrf.DenseCRF2D(img.shape[1], img.shape[0], 2)  # width, height, nlabels
    d.setUnaryEnergy(U)
    # d.addPairwiseGaussian(sxy=sxy_1, compat=compat_1)
    d.addPairwiseBilateral(sxy=sxy, srgb=srgb, rgbim=img, compat=compat) 
    Q = d.inference(5)
    map = np.argmax(Q, axis=0).reshape((img.shape[0],img.shape[1])) 
 
    # filename = 'CRF_Results_1/CRF_mask'
    # filename = filename+'_sxy1'+str(sxy_1)
    # filename = filename+'_compat1'+str(compat_1)
    # filename = filename+'_sxy'+str(sxy)
    # filename = filename+'_srgb'+str(srgb)
    # filename = filename+'_compat'+str(compat)
    # filename = filename + '.jpg' 

    return map


def CRF_refine_road(semantic_mask,image_name, sxy, srgb, compat):

 
    final_mask = np.zeros([semantic_mask.shape[0],semantic_mask.shape[1],2])

    for i in range(semantic_mask.shape[0]):
        for j in range(semantic_mask.shape[1]): 
            # if semantic_mask[i,j,0]+semantic_mask[i,j,2]+semantic_mask[i,j,10]>0.5:
            #     final_mask[i,j,0] = 0.51
            # else:
            #     final_mask[i,j,0] = 0.49
            final_mask[i,j,1] =  semantic_mask[i,j,0]
            final_mask[i,j,0] = 1-final_mask[i,j,1]
            
    
    final_mask = np.moveaxis(final_mask,-1,0)  

    semantic_mask = np.moveaxis(semantic_mask,-1,0)

    

    U = unary_from_softmax(final_mask) 
    img = cv.imread(image_name)  

    print (semantic_mask.shape)
    print (img.shape)

    img = img.copy(order = 'C')
    U = U.copy(order='C')
    U = U.astype(np.float32) 

    sxy_list_1 = np.arange(10,200,50)
    compat_list_1 = np.arange(1,20,5)
    sxy_list = np.arange(10,200,50)
    srgb_list = np.arange(3,14,3)
    compat_list = np.arange(1,20,5)

    # for sxy_1 in sxy_list_1:
    #     for compat_1 in compat_list_1:
    #         for sxy in sxy_list:
    #             for srgb in srgb_list:
    #                 for compat in compat_list:      
    d = dcrf.DenseCRF2D(img.shape[1], img.shape[0], 2)  # width, height, nlabels
    d.setUnaryEnergy(U)
    # d.addPairwiseGaussian(sxy=sxy_1, compat=compat_1)
    d.addPairwiseBilateral(sxy=sxy, srgb=srgb, rgbim=img, compat=compat)
    Q = d.inference(5)
    map = np.argmax(Q, axis=0).reshape((img.shape[0],img.shape[1])) 
 
    # filename = 'CRF_Results_1/CRF_mask'
    # filename = filename+'_sxy1'+str(sxy_1)
    # filename = filename+'_compat1'+str(compat_1)
    # filename = filename+'_sxy'+str(sxy)
    # filename = filename+'_srgb'+str(srgb)
    # filename = filename+'_compat'+str(compat)
    # filename = filename + '.jpg' 

    return map

if __name__ == "__main__":
    seg_map = np.load('seg_map.npy')
    map = CRF_refine(seg_map,'python/video_0.jpg',20,10,5)

    merged_mask = np.zeros(map.shape) 
 
    for i in range(map.shape[0]):
        for j in range(map.shape[1]):  
            if map[i,j] == 1:
                merged_mask[i,j] = 255 
            else:
                merged_mask[i,j] = 0
            

    cv.imwrite("CRF_test.jpg",merged_mask) 
