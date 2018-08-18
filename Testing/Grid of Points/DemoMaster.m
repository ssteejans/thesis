%% Find Centroids of a region
% Sam Artho-Bentz

% Goal is to take a color image, reduce it to only our desired regions, and
% then turn it into black and white so we can use matlab's image processing
% toolbox

%% Clean up
clc
clear all
close all


%% Read in original image
% imread creates a [m,n,3] array where the third index represents values of
% RGB (0-255)
% Explain the 3d array: If a mxn matrix is a sheet of paper filled with
% data, the third dimension is what page of a book we are looking at.
myPhoto = imread('IMG_7849.JPG');

figure(1)   
imshow(myPhoto);    %original image

%% Convert to grayscale
% Converts truecolor RGB to grayscale intensity image
% Weighted sum of the R, G, and B components
MyGSPhoto=rgb2gray(myPhoto);

figure(2)
imshow(MyGSPhoto)   %GS image

%% Filter by color (red)
% subtract GS intensity data from red portion of Original
% removes the background and any non-red elements
myReds=imsubtract(myPhoto(:,:,1), MyGSPhoto);

figure(3)
imshow(myReds)      %Only reds displayed

%% Filter out artifacts
% filter out objects that are less than [3,3] pixels large
m=3;
n=3;
myFiltered=medfilt2(myReds, [m,n]); % 2-D median filtering
        % Each output pixel contains the median 
        % value of all pixels in a mxn area around itself
        % This removes 'salt and pepper noise' from the image. 

figure(4)
imshow(myFiltered);

%% Convert to Binary (black and white)
% Allows use of image processing toolbox for analysis
figure(5)
thres=graythresh(myFiltered);   % graythresh finds appropriate threshold to 
                                % convert grayscale to binary
                                
%myBinary = im2bw(myFiltered, thres); % anything below threshold to 0, 
                                     % above threshold to 1

thresholdValue = 100;
myBinary = MyGSPhoto > thresholdValue;                                    
imshow(myBinary);

%% Remove small areas
% bwareaopen(BW, P) removes connected areas that have fewer than P pixels 
% from the binary image BW
myObjects = bwareaopen(myBinary, 300);
            % Removes from a binary image all connected components(objects)
            % that have fewer than P pixels

figure(6)
imshow(myObjects)

%% Label Objects
% Create labels for connected regions in myObjects (8-connected pixels are 
% neighbors to every pixel that touches one of their edges or corners.)
% These labels are integer values
labels = bwlabel(myObjects, 8); % labels different objects

%% Find Centroid
% region properties returns requested properties for each region labeled in
% 'labels'
% >>doc regionprops  to show them all the options for properties to request
stats = regionprops(labels, 'BoundingBox', 'Centroid'); 
            % Creates a bounding box around 
            % the object and calculates the centroid
            
    % This creates a struct array (open stats and show them that it
    % contains centroid and bounding box) Tell them that to access the data
    % you need to do variable.property
    
    
%% Plot Original Image with Bounding Box and Centroid
figure(7)
imshow(myPhoto);
hold on
for i=1:length(stats)
bc=stats(i).Centroid;
plot(bc(1), bc(2), '-k+');  %-: solid line, k: black, +: plus sign specifer
bb=stats(i).BoundingBox;
rectangle('Position', bb, 'LineWidth', 2.0, 'EdgeColor', 'k');
end
hold off