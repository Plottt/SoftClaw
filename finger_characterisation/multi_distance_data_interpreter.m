close all; clear; clc;

file1 = "multi_distance/2024_1122_0cm.txt";
file2 = "multi_distance/2024_1122_10cm.txt";
file3 = "multi_distance/2024_1122_20cm.txt";

% Plot Force-vs-Angle Data
figure;
subplot(3,1,1);
plot_force_angle(file1, "0cm Offset", true);
subplot(3,1,2);
plot_force_angle(file2, "10cm Offset", false);
subplot(3,1,3);
plot_force_angle(file3, "20cm Offset", false);

% Plot angle, angular acceleration, and force for contact detection
figure; 
plot_contact_detection(file3, 38); % clearest results: 20cm distance at 20% duty cycle (38/255)

function plot_force_angle(file_name, plot_title, show_legend)
    data = readmatrix(file_name);
    
    data = data(10:end, :); % some garbage data can get recorded at the beginning
    
    time_us = data(:, 1);
    time_s = time_us / 1000000;
    time_s(1)
    time_s = time_s - time_s(1);
    pwm = data(:, 2);
    x_pos_deg = data(:, 3);
    y_pos_deg = data(:, 4);
    force_grams = data(:, 5);
    force_mN = force_grams .* 9.80665;
    
    pwms = unique(pwm); %     25    38    51    64    76    89   102   115   127
    % data_by_pwm = struct(); % init a structure
    
    cmap = turbo(10);
    % cmap = cmap ./1.2;
    hold on;
    for i = 1:length(pwms)
        pwm_val = pwms(i);
        if (pwm_val == 38 || pwm_val == 76 || pwm_val == 115) % for clarity only plot 15, 30, 45 %s
            idx = pwm == pwm_val; % index for the current PWM value
        
        
            angles_for_pwm_level = x_pos_deg(idx);
            angles_for_pwm_level = angles_for_pwm_level(3:end); % remove some garbage data at the beginning
            forces_for_pwm_level = force_mN(idx);
            forces_for_pwm_level = forces_for_pwm_level(3:end); % remove some garbage data at the beginning
            times_for_pwm_level = time_s(idx);
            times_for_pwm_level = times_for_pwm_level(3:end);
            pwms_for_pwm_level = pwm(idx);
            pwms_for_pwm_level = pwms_for_pwm_level(3:end);
        
          
        
            [unique_angles, unique_idxs] = unique(angles_for_pwm_level);
        
            unique_forces = forces_for_pwm_level(unique_idxs);
            
            % % Store data in structure
            % data_by_pwm.(sprintf('pwm_%d', pwm_val)).force_mN = unique_forces;
            % data_by_pwm.(sprintf('pwm_%d', pwm_val)).angles_deg = unique_angles;

            plot(unique_angles, unique_forces, LineWidth=2, Color=cmap(i+1,:));
        end

    
    end
    
    xlim([0,30]);
    ylim([-10,450]);
    xlabel('Angle [deg]', 'FontSize',11);
    ylabel('Force [mN]', 'FontSize',11);
    % if(show_legend)
    %     legend({'10% Duty Cycle', '15% Duty Cycle', '20% Duty Cycle', '25% Duty Cycle', ...
    %         '30% Duty Cycle', '35% Duty Cycle', '40% Duty Cycle', '45% Duty Cycle', '50% Duty Cycle'});
    % end
    if(show_legend)
        legend({'15% Duty Cycle', '30% Duty Cycle', '45% Duty Cycle'}, 'FontSize',10);
    end
    t = title(plot_title, 'FontSize', 12); 
    disp(t.Position(2))
    t.BackgroundColor = 'w';  % Set the background to white
    t.Position(2) = 350;   % Move the title down 
    grid on;
end


function plot_contact_detection(file_name,duty_cycle)
    data = readmatrix(file_name);
    
    data = data(10:end, :); % some garbage data can get recorded at the beginning
    
    time_us = data(:, 1);
    time_s = time_us / 1000000;
    time_s(1)
    time_s = time_s - time_s(1);
    pwm = data(:, 2);
    x_pos_deg = data(:, 3);
    y_pos_deg = data(:, 4);
    force_grams = data(:, 5);
    force_mN = force_grams .* 9.80665;

    pwms = unique(pwm); %     25    38    51    64    76    89   102   115   127
    % data_by_pwm = struct(); % init a structure

    hold on;
    for i = 1:length(pwms)
        pwm_val = pwms(i);
        if (pwm_val == duty_cycle)
            idx = pwm == pwm_val; % index for the current PWM value
        
        
            angles_for_pwm_level = x_pos_deg(idx);
            angles_for_pwm_level = angles_for_pwm_level(3:end); % remove some garbage data at the beginning
            forces_for_pwm_level = force_mN(idx);
            forces_for_pwm_level = forces_for_pwm_level(3:end); % remove some garbage data at the beginning
            times_for_pwm_level = time_s(idx);
            times_for_pwm_level = times_for_pwm_level(3:end);
            pwms_for_pwm_level = pwm(idx);
            pwms_for_pwm_level = pwms_for_pwm_level(3:end);
        
    
            [unique_angles, unique_idxs] = unique(angles_for_pwm_level);
        
            unique_forces = forces_for_pwm_level(unique_idxs);
            unique_times = times_for_pwm_level(unique_idxs);
            
            % % Store data in structure
            % data_by_pwm.(sprintf('pwm_%d', pwm_val)).time_s = unique_times;
            % data_by_pwm.(sprintf('pwm_%d', pwm_val)).force_mN = unique_forces;
            % data_by_pwm.(sprintf('pwm_%d', pwm_val)).angles_deg = unique_angles;

            adjusted_times = unique_times-unique_times(1);
            accel = gradient(gradient(unique_angles, unique_times), unique_times);
            % Plot Angles
            subplot(3,1,1); hold on;
            plot(adjusted_times(1:30), unique_angles(1:30), 'LineWidth', 2, 'HandleVisibility', 'off');
            plot([4.45, 4.45], [0, 22], 'r--', 'LineWidth', 2, 'DisplayName', 'Contact');
            xlim([0, 9]);
            legend();
            grid on;
            title('Measured from Finger')
            ylabel({'bend angle', '[deg]'}, 'FontSize',14);
            % Plot acceleration (d^2Angle/dt^2)
            subplot(3,1,2); hold on;
            plot(adjusted_times(5:30), accel(5:30), LineWidth=2);
            plot([4.45, 4.45], [-4,2], 'r--', LineWidth=2)
            xlim([0, 9])
            title('Calculated from Finger');
            grid on;
            ylabel({'bend accel.', '[deg/s^2]'}, 'FontSize',14);
            % Plot recorded forces from the load cell
            subplot(3,1,3); hold on;
            plot(adjusted_times(1:30), unique_forces(1:30), LineWidth=2);
            plot([4.45, 4.45], [0,200], 'r--', LineWidth=2)
            xlim([0, 9])
            grid on;
            title('Measured from Load Cell')
            ylabel("Force [mN]", 'FontSize',14);
            xlabel("Time [s]", 'FontSize',14);

        end  
    end    
end
