%% EGH445 - Controller Gain 'K' Search

% syms k0 k1 k2 k3 s
% l1 = -3; l2 = -4; l3 = -5; l4 = -6;
% ladd = l1 + l2 + l3+ l4;
% l1l2 = l1*l2;
% l1l3 = l1*l3;
% l1l4 = l1*l4;
% l2l3 = l2*l3;
% l2l4 = l2*l4;
% l3l4 = l3*l4;
% l1l2l3 = l1*l2*l3;
% l1l2l4 = l1*l2*l3;
% l1l3l4 = l1*l3*l4;
% l2l3l4 = l2*l3*l4;
% l1l2l3l4 = l1*l2*l3*l4;
% K = [k0 k1 k2 k3];
% [k0, k1, k2, k3] = solve(s^4 - ladd*s^3 + (l1l2+l1l3+l1l4+l2l3+l2l4+l3l4)*s^2 ...
%     - (l1l2l3 + l1l2l4 + l1l3l4)*s + l1l2l3l4 == det(s*eye(size(A)) - (A - B*K)));
% K = [double(k0) double(k1) k2 k3];

tot = inf;
for l1 = -10:1:-1
    for l2 = -10:1:-1
        for l3 = -10:1:-1
            for l4 = -10:1:-1
                
                K = place(A, B, [l1, l2, l3, l4]);
                
                totr = sum(K);
                if totr < tot
                    tot = totr;
                    bestK = K;
                    bestL = [l1 l2 l3 l4];
                end
                
            end
            
        end
        
    end
end



% (s-l1)*(s-l2)*(s-l3)*(s-l4)

% for k0 = 1:1:20
%     for k1 = 1:1:20
%         
%         
%         
%         
%     end
% end
