function B = transfer(S,B1,L1,M,alpha,errorTol)

% Define error tolerance.
if ~exist('errorTol','var')
   errorTol = 0.1;
end

% Determine block indices.
blockRows = 0:size(B1,1)-1;
blockCols = 0:size(B1,2)-1;

% Perform exhaustive search to determine similar blocks.
E = zeros(1,prod(S.nblocks));
for i = 1:prod(S.nblocks)
   B2 = S.image(S.blockIndex(i,1)+blockRows,S.blockIndex(i,2)+blockCols,:);
   L2 = S.L(S.blockIndex(i,1)+blockRows,S.blockIndex(i,2)+blockCols,:);
   E(i) = alpha*sum((B1(M)-B2(M)).^2) + (1-alpha)*sum((L1(:)-L2(:)).^2);
end

% Randomly select output block amoung matches.
match_index = find(E <= (1+errorTol^2)*min(E));
i = ceil(length(match_index)*rand);
B = S.image(S.blockIndex(match_index(i),1)+blockRows,...
            S.blockIndex(match_index(i),2)+blockCols,:);