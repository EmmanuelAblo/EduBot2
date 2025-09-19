image = imread('tag_36h11.png');
resized_image = imresize(image, [1024  512]);
imwrite(resized_image, 'tag_36h11_resized.png');
