function [A, B] = IdentARX(u, y, na, nb, nk, Ta)
    % Reshape data to column vectors
    y = reshape(y,[length(y),1]);
    u = reshape(u,[length(y),1]);

    % Build output vector
    offset = max(na,nb+nk);
    Y = y(offset+1:end);

    % Build data matrix
    Phi = zeros([length(Y), na+nb+1]);
    for col = 1:na+nb+1      
        if(col <= na)
            % Fill first na columns with past outputs
            Phi(:,col) = -y((offset - col) + (1:length(Y)));
        else
            % Fill last nb columns with past inputs shifted by nk
            col_ = col - na;
            Phi(:,col) = u((offset + 1 - nk - col_) +  (1:length(Y)));
        end
    end

    % Compute parameters using least squares
    Theta = pinv(Phi)*Y;

    % Fill denominator coefficients
    A = ones(na+1,1);
    A(2:na+1) = Theta(1:na);
    
    % Fill nominator coefficients
    B = zeros(nb+nk+1,1);
    B(nk+1:nk+nb+1) = Theta(na+1:end);
    
    % Transpose to be able to directly use in tf-function
    A = A';
    B = B';

    % Build tf
    tf(B,A, Ta, 'Variable', 'z');
end