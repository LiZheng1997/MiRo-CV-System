import cv2
import numpy as np
import random
 #180秒训练完
 
def load_images(dirname, amout = 9999):
    img_list = []
    file = open(dirname)
    img_name = file.readline()
    while img_name != '':  # 文件尾
        img_name = dirname.rsplit(r'/', 1)[0] + r'/' + img_name.split('/', 1)[1].strip('\n')
        img_list.append(cv2.imread(img_name))
        img_name = file.readline()
        amout -= 1
        if amout <= 0: # 控制读取图片的数量
            break
    return img_list
 
 
# 从每一张没有人的原始图片中随机裁出10张64*128的图片作为负样本
def sample_neg(full_neg_lst, neg_list, size):
    random.seed(1)
    width, height = size[1], size[0]
    for i in range(len(full_neg_lst)):
        for j in range(10):
            y = int(random.random() * (len(full_neg_lst[i]) - height))
            x = int(random.random() * (len(full_neg_lst[i][0]) - width))
            neg_list.append(full_neg_lst[i][y:y + height, x:x + width])
    return neg_list
 
 
# wsize: 处理图片大小，通常64*128; 输入图片尺寸>= wsize
def computeHOGs(img_lst, gradient_lst, wsize=(128, 64)):
    hog = cv2.HOGDescriptor()
    # hog.winSize = wsize
    for i in range(len(img_lst)):
        if img_lst[i].shape[1] >= wsize[1] and img_lst[i].shape[0] >= wsize[0]:
            roi = img_lst[i][(img_lst[i].shape[0] - wsize[0]) // 2: (img_lst[i].shape[0] - wsize[0]) // 2 + wsize[0], \
                  (img_lst[i].shape[1] - wsize[1]) // 2: (img_lst[i].shape[1] - wsize[1]) // 2 + wsize[1]]
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            gradient_lst.append(hog.compute(gray))
    # return gradient_lst
 
 
def get_svm_detector(svm):
    sv = svm.getSupportVectors()
    rho, _, _ = svm.getDecisionFunction(0)
    sv = np.transpose(sv)
    return np.append(sv, [[-rho]], 0)
 
 
# 主程序
# 第一步：计算HOG特征
neg_list = []
pos_list = []
gradient_lst = []
labels = []
hard_neg_list = []
svm = cv2.ml.SVM_create()
pos_list = load_images(r'/home/lizheng/opencv_demos/INRIAPerson/train_64x128_H96/pos.lst')
full_neg_lst = load_images(r'/home/lizheng/opencv_demos/INRIAPerson/train_64x128_H96/neg.lst')
sample_neg(full_neg_lst, neg_list, [128, 64])
print(len(neg_list))
computeHOGs(pos_list, gradient_lst)
[labels.append(+1) for _ in range(len(pos_list))]
computeHOGs(neg_list, gradient_lst)
[labels.append(-1) for _ in range(len(neg_list))]
 
# 第二步：训练SVM
svm.setCoef0(0)
svm.setCoef0(0.0)
svm.setDegree(3)
criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 1000, 1e-3)
svm.setTermCriteria(criteria)
svm.setGamma(0)
svm.setKernel(cv2.ml.SVM_LINEAR)
svm.setNu(0.5)
svm.setP(0.1)  # for EPSILON_SVR, epsilon in loss function?
svm.setC(0.01)  # From paper, soft classifier
svm.setType(cv2.ml.SVM_EPS_SVR)  # C_SVC # EPSILON_SVR # may be also NU_SVR # do regression task
svm.train(np.array(gradient_lst), cv2.ml.ROW_SAMPLE, np.array(labels))
 
# 第三步：加入识别错误的样本，进行第二轮训练
# 参考 http://masikkk.com/article/SVM-HOG-HardExample/
hog = cv2.HOGDescriptor()
hard_neg_list.clear()
hog.setSVMDetector(get_svm_detector(svm))
for i in range(len(full_neg_lst)):
    rects, wei = hog.detectMultiScale(full_neg_lst[i], winStride=(4, 4),padding=(8, 8), scale=1.05)
    for (x,y,w,h) in rects:
        hardExample = full_neg_lst[i][y:y+h, x:x+w]
        hard_neg_list.append(cv2.resize(hardExample,(64,128)))
computeHOGs(hard_neg_list, gradient_lst)
[labels.append(-1) for _ in range(len(hard_neg_list))]
svm.train(np.array(gradient_lst), cv2.ml.ROW_SAMPLE, np.array(labels))
 
 
# 第四步：保存训练结果
hog.setSVMDetector(get_svm_detector(svm))
hog.save('myHogDector.bin')
