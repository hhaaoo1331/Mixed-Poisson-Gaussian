 function D= D(U)
            % Forward finite difference operator
            Duy = [diff(U,1,2), U(:,1) - U(:,end)];
            Dux = [diff(U,1,1); U(1,:) - U(end,:)];
            D = [Dux,Duy];