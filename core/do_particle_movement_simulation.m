function [x, y, vx, vy] = do_particle_movement_simulation(x, y, vx, vy, radius_vector, mass_vector, x_bound, y_bound)
%DO_PARTICLE_MOVEMENT_SIMULATION main simulation function, do all maths 
%   x,y - particles positions vector
%   vx,vy - particles velocity components
%   x_bound,y_bound - box bonduaries
%   radius_vector - particles radius vector
%   mass_vector - particles mass vector
%   in both (radius_vector and mass_vector) index in array is particle id

    dt = 0.05;
    particle_count = numel(x);
    already_collided = zeros(particle_count, particle_count);
    xf = x + dt.*vx;
    yf = y + dt.*vy;
    for j=1:1:particle_count
        d = calculate_particles_distance(xf(j),yf(j),xf,yf);
        for k=1:1:particle_count
            if(d(k) <= radius_vector(k)+radius_vector(j) && k~=j && already_collided(j,k)~=1 && already_collided(k,j)~=1)
                [vx(k),vy(k),vx(j),vy(j)]=collide_particles(mass_vector(k),x(k),y(k),vx(k),vy(k),mass_vector(j),x(j),y(j),vx(j),vy(j));
                already_collided(j,k)=1;
                already_collided(k,j)=1;
            end
        end
        if(x(j)+radius_vector(j) >= x_bound || x(j)-radius_vector(j) <=0)
            if(x(j)-radius_vector(j) <=0)
                x(j) = x(j)+(abs(0-x(j)+radius_vector(j)));
            else
                x(j) = x(j)-(x(j)+radius_vector(j) - x_bound);
            end
            vx(j) = -vx(j);
        end
        
        if(y(j)+radius_vector(j) >= y_bound  || y(j)-radius_vector(j) <=0)
            if(y(j)-radius_vector(j) <=0)
                y(j) = y(j)+(abs(0-y(j)+radius_vector(j)));
            else
                y(j) = y(j)-(y(j)+radius_vector(j) - y_bound);
            end
            vy(j) = -vy(j);
        end
        
    end
    x = x + dt.*vx;
    y = y + dt.*vy;
end

