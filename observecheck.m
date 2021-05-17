%% EGH445 Observability Check

Osc = obsv(A, C);

if (rank(Osc) == size(A, 2))
    
    fprintf('Equilibrium Point "%s" is Completely Observable \n', EP);
    
else
    
    fprintf('Equilibrium Point "%s" is NOT Completely Observable \n', EP);
    
end
