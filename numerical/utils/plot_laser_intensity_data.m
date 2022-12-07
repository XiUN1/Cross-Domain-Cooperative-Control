function f = plot_laser_intenisty_data(samples, states, fig_num)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

all_states = [states{:}];
intensity_states = {all_states.laser_intensity};

intensity_vector = [];

for ii = 1:length(intensity_states)
    % for each time sample
    intensities = intensity_states{ii}';

    for jj = 1:size(intensities, 2)
        % for each laser intensity signal
        intensities_jj = intensities(jj);
        if jj == 1
            % ugly hackery
            intensity_vector(jj,end+1) = intensities_jj(1);
        else
            intensity_vector(jj,end) = intensities_jj(1);
        end
        
    end
    
end

f = figure(fig_num);
plot(samples, intensity_vector, 'LineWidth', 1.5); grid on; xlabel("Time [s]"); ylabel("Laser Intensity [Watts]")
end