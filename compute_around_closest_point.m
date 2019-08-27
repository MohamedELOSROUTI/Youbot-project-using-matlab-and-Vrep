function closestPoint = compute_around_closest_point(center,r,youbotPos_map,N,k)
% compute the closest point around a center( of table or basket) from
% youbot position
% center : coordinate of the table/basket
% k : the k'nd closest distance.
% r : radius along which the compute the points that surround the
% table/basket

    radiusFromCenter = [0;r] ; % set to 1 > radTable (because inflate)
    Rot = [cosd(360/N), -sind(360/N);...
           sind(360/N), cosd(360/N)];
    aroundCenterTablePoints = radiusFromCenter;
    for j = 2:N
        aroundCenterTablePoints(:,j) = Rot*...
                          aroundCenterTablePoints(:,j-1);
    end
    aroundCenterTablePoints1 = aroundCenterTablePoints + center';
    
    distances1 = zeros(N,1); % distance from youbot to table 1
    
    for j = 1:N
        distances1(j) = norm(aroundCenterTablePoints1(:,j)-youbotPos_map(1:2)');
        
    end
%     indexMin1 = distances1 == min(distances1); % closest point to table 1 from youbot
%     closestPoint = aroundCenterTablePoints1(:,indexMin1)';
    [~,indicesMin1] = sort(distances1);
    closestPoint = aroundCenterTablePoints1(:,indicesMin1(k))';
    
end