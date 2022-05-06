function  Dt = Dt(t)
            % Transpose of the forward finite difference operator
            [m,n]=size(t);
            Y = t(1:m,1:n/2);
            X= t(1:m,n/2+1:n);
            Dt = [X(:,end) - X(:, 1), -diff(X,1,2)];
            Dt = Dt + [Y(end,:) - Y(1, :); -diff(Y,1,1)];