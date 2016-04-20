filename = 'example.csv';
im1 = imread('../images/im1.png');
im2 = imread('../images/im2.png');
image(im1);         %# Any subsequent plotting will overwrite the image!
figure;
image(im2);
figure;
M = csvread(filename);
scatter3(M(:,1),M(:,2),M(:,3));

% Add title and axis labels
title('Ozone Levels')
xlabel('X')
ylabel('Y')
zlabel('Z')

im3 = imread('../images/epi_left.jpg');
figure;
image(im3);
