function [x,y] = initialize_particle_box(particle_count, particle_radius_vector, particle_mass_vector, x_bound, y_bound, color_map)
%INITIALIZE_PARTICLE_BOX function initiaze particles positions and draw
%particles in current axes
%   particle_count - number of particles in box
%   particle_radius_vector - each particle radius
%   particle_mass_vector - each particle mass
%   x_bound, y_bound - box bounduary
%   color_map - color map of particles
x=zeros(1,particle_count);
y=zeros(1,particle_count);
particle_added = 0;
while (particle_added < particle_count)
    current_particle_radius = particle_radius_vector(particle_added+1);
    collision_count = 0 ;
    random_x = (x_bound-5*current_particle_radius)*rand+3*current_particle_radius;
    random_y = (y_bound-5*current_particle_radius)*rand+3*current_particle_radius;
    particle_distance = calculate_particles_distance(random_x,random_y,x,y);

    for i=1:particle_count
        if current_particle_radius + particle_radius_vector(i) > particle_distance(i) && i~=j
            collision_count = collision_count + 1;
        end        
    end
    
    if(collision_count == 0)
        x(particle_added+1) = random_x;
        y(particle_added+1) = random_y;
        particle_added = particle_added+1;
    end
end

end


