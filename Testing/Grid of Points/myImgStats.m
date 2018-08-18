function centroids = myImgStats(myPhoto,mask)

myGrey = rgb2gray(myPhoto);

myMask = imsubtract(myPhoto(:,:,mask),myGrey);

myFiltered = medfilt2(myMask,[3 3]);

thres = graythresh(myFiltered);

myBinary = im2bw(myFiltered,thres);

myObjects = bwareaopen(myBinary,300);

labels = bwlabel(myObjects,8);

stats = regionprops(labels,'BoundingBox','Centroid');

centroids = cat(1, stats.Centroid);

