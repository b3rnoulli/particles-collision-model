function [d] = calculate_particles_distance( x1, y1, x2, y2 )
%CALCULATE_PARTICLES_DISTANCE - simple euclidan distance between particles
%- calculated manualy becouse of performance

     x1 = ones(1, numel(x2))*x1;
     y1 = ones(1, numel(y2))*y1;
     d = sqrt((x1-x2).^2 + (y1-y2).^2); 

end

