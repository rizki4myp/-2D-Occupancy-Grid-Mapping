% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping_old(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% % the number of grids for 1 meter.
% myResol = param.resol;
% % the initial map size in pixels
% myMap = zeros(param.size);
% % the origin of the map in pixels
% myorigin = param.origin; 
% 
% % 4. Log-odd parameters 
% lo_occ = param.lo_occ;
% lo_free = param.lo_free; 
% lo_max = param.lo_max;
% lo_min = param.lo_min;

myResol = param.resol;
myMap = zeros(param.size);
myorigin = param.origin; 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

N = size(pose,2);
for j = 1:N % for each time,
    
     % Find grids hit by the rays (in the gird map coordinate)
     occ_real = ([ranges(:,j).*cos(scanAngles+pose(3,j)),...
         -ranges(:,j).*sin(scanAngles+pose(3,j))] +repmat([pose(1,j),pose(2,j)],size(scanAngles,1),1))';% [x_occ;y_occ]
     occ_sub = ceil(myResol*occ_real) + repmat(myorigin,1,size(scanAngles,1));% [x_occ;y_occ],
     rob_sub = ceil(myResol*[pose(1,j);pose(2,j)]) + myorigin;
     % Find occupied-measurement cells and free-measurement cells
     occ_index = sub2ind(size(myMap),occ_sub(2,:),occ_sub(1,:)); 
     for i = 1:size(scanAngles,1)
         % start point is the point of the robot
        [freex, freey] = bresenham(rob_sub(1),rob_sub(2),occ_sub(1,i),occ_sub(2,i));
        free_index = sub2ind(size(myMap),freey,freex); 

        myMap(free_index) = myMap(free_index) - lo_free;
     end
     
     % Update the log-odds
     myMap(occ_index) = myMap(occ_index) + lo_occ;
     
     % Saturate the log-odd values
     % reassign value if out of range
     
     % Visualize the map as needed
    
 
 end
myMap(myMap < lo_min) = lo_min;
myMap(myMap > lo_max) = lo_max;
end

