function [W,dist] = align_vectors(v,W)
% Alings the row-vectors in the matrix "W" (m-x-n) with the row vector "v" (m-x-1)
m = size(W,2);
dist = zeros(1,m);
[~,idx] = max(v); % idx_max is the most dominant index in "v"
% v = v.*exp(-1j*angle(v(idx))); % rotate so that index "idx" has angle = 0 (should already have been done)
for i = 1:m
    % Align position "idx" in both "v" and "W(:,i)"
    W(:,i) = W(:,i).*exp(-1j*angle(W(idx,i))); 
    dist(i) = norm(W(:,i)-v);
end

end

