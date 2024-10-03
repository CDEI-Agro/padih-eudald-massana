# https://youtu.be/ieyODuIjXp4
"""
@author: Sreenivas Bhattiprolu

Download and install GDAL first
https://www.lfd.uci.edu/~gohlke/pythonlibs/#gdal
Since I have python 3.7 I downloaded: GDAL-3.1.4-cp37-cp37m-win_amd64.whl
cp37 stands for python3.7

Download and install rasterio
https://www.lfd.uci.edu/~gohlke/pythonlibs/#rasterio
Since I have python 3.7 I downloaded: rasterio-1.1.8-cp37-cp37m-win_amd64.whl
cp37 stands for python3.7

Images from: https://landsatonaws.com/L8/042/034/LC08_L1TP_042034_20180619_20180703_01_T1



"""

import rasterio
from rasterio.plot import show
from matplotlib import pyplot as plt
from matplotlib import image as im
import utm
import cv2 as cv

img = rasterio.open('C:\Dades\Projectes\Actius\Rovinya\Mapes\maset_2.tif')
#show(img)
#X and Y are supposed to be latitude and longitude if you have the right metadata
print(img.bounds)

u = utm.from_latlon(41.381224, 1.780866) # coordenades fixes Eudald
print(u)
print(img.index(u[0], u[1]))
# latlon = utm.to_latlon(*u)
# print(latlon)
# print(img.transform)

full_img = img.read()  #Note the 3 bands and shape of image
print (full_img.shape)
#ima = cv.imread('C:\Dades\Projectes\Actius\Rovinya\Mapes\maset.tif')
trima = full_img.transpose((1,2,0)) #passem de (H, W, D) a (D, H, W)
print (trima.shape)
plt.imshow(trima)

#cv.cvtColor(trima,cv.COLOR_RGB2BGR)
cv.imwrite('C:\Dades\Projectes\Actius\Rovinya\Mapes\maset.jpg',cv.cvtColor(trima,cv.COLOR_RGB2BGR))
#plt.savefig('C:\Dades\Projectes\Actius\Rovinya\Mapes\maset.jpg')

# cv.imwrite('ima1.jpg',full_img)


#To find out number of bands in an image
num_bands = img.count
print("Number of bands in the image = ", num_bands)

img_band1 = img.read(1) #1 stands for 1st band. 
img_band2 = img.read(2) #2 stands for 2nd band. 
img_band3 = img.read(3) #3 stands for 3rd band. 

