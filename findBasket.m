clear all
close all
clc

load('mapping_end2');
imap_ = occupancyMatrix(imap, 'ternary');
map_ = occupancyMatrix(map, 'ternary');
radiusRange = 0.8;
figure(1)
imshow(map_)
[centers, radii, metric] = imfindcircles(imap_, [10 13], 'ObjectPolarity', 'bright', 'Sensitivity', 1);
centersStrong5 = centers(1:7,:);
radiiStrong5 = radii(1:7);
metricStrong5 = metric(1:7);
viscircles(centersStrong5, radiiStrong5, 'EdgeColor', 'b');
idx = sub2ind(size(imap_),centersStrong5(:,1),centersStrong5(:,2));
idx = int64(idx);
map_(idx) = 1000;
imshow(map_)
% metrics : > 0.05