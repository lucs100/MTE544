clear
load datatables.mat

data = laser_filtered_content_line;

points = zeros(360*height(data), 3);

for i = 1:height(data)
    myRow = table2array(data(i,1:2:720)); % Data is in min-max pairs
    theta = linspace(0, 2*pi, 360);
    points(1 + (i-1)*360 : i * 360, 1) = cos(theta).*myRow;
    points(1 + (i-1)*360 : i * 360, 2) = sin(theta).*myRow;
    points(1 + (i-1)*360 : i * 360, 3) = i;
end

scatter3(points(:, 1), points(:, 2), rescale(points(:, 3))*100, 1, points(:, 3))
xlabel("Distance [m]")
ylabel("Distance [m]")
zlabel("Scenario progress [%]")
title("LaserScan - Line")