%% EGH445 Observability Check

Oac = obsv(A, C);

if (rank(Oac) == size(A, 2))
    
    fprintf('Equilibrium Point "%s" is Completely Observable \n', EP);
    
else
    
    fprintf('Equilibrium Point "%s" is NOT Completely Observable \n', EP);
    
end
