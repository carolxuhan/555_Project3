function B = findmatch(S,B1,M,preCache,errorTol)

% Define error tolerance and pre-caching state.
if ~exist('preCache','var')
   preCache = false;
end
if ~exist('errorTol','var')
   errorTol = 0.1;
end

% Vectorize search block (using input mask).
blockRows = 0:size(B1,1)-1;
blockCols = 0:size(B1,2)-1;
B1 = B1(M);

% Perform exhaustive search to determine similar blocks.
E = zeros(1,prod(S.nblocks));
if preCache
   E = sum((repmat(B1,1,prod(S.nblocks))-S.blocks(M,:)).^2,1);
else
   for i = 1:prod(S.nblocks)
      B2 = S.image(S.blockIndex(i,1)+blockRows,S.blockIndex(i,2)+blockCols,:);
      E(i) = sum((B1-B2(M)).^2);
   end
end

% Randomly select output block amoung matches.
match_index = find(E <= (1+errorTol^2)*min(E));
i = ceil(length(match_index)*rand);
B = S.image(S.blockIndex(match_index(i),1)+blockRows,...
            S.blockIndex(match_index(i),2)+blockCols,:);