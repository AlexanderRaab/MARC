function [K,T, MSE] = IdentLag1(t, u, y, perc)

    % Prune data to relevant range
    idx_s = find(u > 0,1,'first');                    % First index of step response
    idx_e = find(u(idx_s:end) ~= u(idx_s),1,'first');  % Last index of step response
    if(isempty(idx_e)) 
        idx_e = length(t);
    end

    t = t(idx_s:idx_e-1);
    u = u(idx_s:idx_e-1);
    y = y(idx_s:idx_e-1);
    
    % Plot data
    if(perc == 0)
        figure;
        plot(t,u,'DisplayName','u', Color='red');
        legend;
        hold on;
        plot(t,y,'DisplayName','y', Color='blue');
    end
 
    % Identify static gain
    if(perc == 0)
        % Pick 2 points where final state is reached
        [t1,y1] = ginput(1);
        idx_1 = find(t >= t1,1,'first');
        if(isempty(idx_1)) 
            idx_1 = length(t);
        end
        text(t(idx_1) , y(idx_1), '+');
    
        [t2,y2] = ginput(1);
        idx_2 = find(t >= t2, 1,'first');
        if(isempty(idx_2)) 
            idx_2 = length(t);
        end
        text(t(idx_2) , y(idx_2), '+');
    else
        % Use input percentages
        idx_1 = round(length(t)*perc)
        idx_2 = length(t)
    end

    % Calculate static gain K
    y_s = mean(y(idx_1:idx_2));
    K = y_s/u(idx_s);

    % Calculate time constant T 
    % Use area under curve to minimize impact of sample noise
    area = 0;
    for i = 1:length(t)-1
        area = area + (y(i+1)+y(i))/2*(t(i+1)-t(i));
    end
    % Use: y_s*(t_e - t_s) - area = T * y_s
    T = (t(end) - t(1)) - area / y_s;

    % Plot step respone of identified system
    y_ident = u(idx_s)*K*(1-exp(-t/T));
    if(perc == 0)
        plot(t, y_ident, 'DisplayName','y_{ident}',Color='black');
    end
    
    % Mean squared error
    MSE = mean((y - y_ident).^2);
end