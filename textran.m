
% Reset Matlab environment.
clear; clc; randseed(0);

% Set Image Quilting (IQ) parameters.
sourceImage  = './textures/examples/mesh.jpg'; % source image
blockSize    = [10 10];                  % block size
blockOverlap = ceil(blockSize/6);        % block overlap
errorTol     = 0.1;                      % matching-error tolerance
bndCut       = true;                     % enable/disable boundary optimization

% Set texture transfer parameters.
targetImage  = './images/girl.jpg';      % target image (for texture transfer)
alpha        = 0.2;                      % texture/correspondence trade-off

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Pre-process source texture (to extract individual blocks for synthesis).

% Load source texture.
S.image    = imresize(im2double(imread(sourceImage)),0.5);
S.nrows    = size(S.image,1);
S.ncols    = size(S.image,2);
S.ncolors  = size(S.image,3);
S.nblocks  = [S.nrows S.ncols]-blockSize+1;

% Extend to RBG (if grayscale).
if S.ncolors == 1
   S.image = repmat(S.image,[1 1 3]);
   S.ncolors = size(S.image,3);
end
   
% Determine luminance of source image.
if S.ncolors == 1
   S.L = S.image;
else
   S.L = 0.2990*S.image(:,:,1) + ...
         0.5870*S.image(:,:,2) + ...
         0.1140*S.image(:,:,3);
end
S.L = imfilter(S.L,fspecial('Gaussian',10,2));

% Display source texture.
figure(1); clf; imagesc(S.image);
set(gcf,'Name','Source Texture');
axis image; drawnow;

% Verify block size and overlaps.
if (blockSize(1) > S.nrows) || (blockSize(2) > S.ncols)
   error(['The output texture does not support ',...
      int2str(blockSize(1)),'x',int2str(blockSize(2)),' blocks.']);
end
if (blockOverlap(1) >= blockSize(1)) || (blockOverlap(2) >= blockSize(2))
   error(['A ',int2str(blockSize(1)),'x',int2str(blockSize(2)),...
      ' block does not support ',...
      int2str(blockOverlap(1)),'x',int2str(blockOverlap(2)),' overlaps.']);
end

% Determine locations of source blocks.
[C,R] = meshgrid(1:S.nblocks(2),1:S.nblocks(1));
R = R'; C = C'; S.blockIndex = [R(:) C(:)];
blockRows = 0:blockSize(1)-1;
blockCols = 0:blockSize(2)-1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Apply Image Quilting (IQ) to generate a synthetic texture.

% Allocate storage for the synthesized output image/mask.
% Note: Increase size to prevent clipped blocks at edges.
%       Remember to crop output image after synthesis.
overlapSize = blockSize-blockOverlap;
trgImg    = imresize(im2double(imread(targetImage)),0.75);
T.nrows   = overlapSize(1)*(ceil(size(trgImg,1)/overlapSize(1))-1)+blockSize(1);
T.ncols   = overlapSize(2)*(ceil(size(trgImg,2)/overlapSize(2))-1)+blockSize(2);
T.ncolors = size(trgImg,3);
T.image   = imresize(trgImg,[T.nrows T.ncols],'bilinear');
T.nblocks = floor([T.nrows T.ncols]./overlapSize);
T.mask    = false(T.nrows,T.ncols,T.ncolors);
T.cut     = false(T.nrows,T.ncols);

% Extend to RBG (if grayscale).
if T.ncolors == 1
   T.image = repmat(T.image,[1 1 3]);
   T.ncolors = size(T.image,3);
end

% Determine luminance of target image.
if T.ncolors == 1
   T.L = T.image;
else
   T.L = 0.2990*T.image(:,:,1) + ...
         0.5870*T.image(:,:,2) + ...
         0.1140*T.image(:,:,3);
end
T.L = imfilter(T.L,fspecial('Gaussian',10,1));

% Determine locations of overlapping output blocks.
[C,R] = meshgrid(overlapSize(2)*(0:T.nblocks(2)-1)+1,...
                 overlapSize(1)*(0:T.nblocks(1)-1)+1);
R = R'; C = C'; T.blockIndex = [R(:) C(:)];

% Seed the synthesized texture with a block of similar luminance.
E = zeros(1,prod(S.nblocks));
L2 = T.L(blockRows+1,blockCols+1,:);
for i = 1:prod(S.nblocks)
   L1 = S.L(S.blockIndex(i,1)+blockRows,...
            S.blockIndex(i,2)+blockCols,:);
   E(i) = sum((L1(:)-L2(:)).^2);
end
[ignore,i] = min(E);
B = S.image(S.blockIndex(i,1)+blockRows,...
            S.blockIndex(i,2)+blockCols,:);
T.image(blockRows+1,blockCols+1,:) = B;
T.mask(blockRows+1,blockCols+1,:) = 1;
% i = ceil(prod(S.nblocks)*rand);
% B = S.image(S.blockIndex(i,1)+blockRows,...
%             S.blockIndex(i,2)+blockCols,:);
% T.image(blockRows+1,blockCols+1,:) = B;
% T.mask(blockRows+1,blockCols+1,:) = 1;

% Display synthesized texture.
figure(2); clf; imagesc(T.image);
set(gcf,'Name','Synthesized Texture');
axis image; drawnow;

% Synthesize the remaining pixels using IQ.
for i = 2:prod(T.nblocks)

   % Extract current value (i.e., the overlapping region).
   B1 = T.image(T.blockIndex(i,1)+blockRows,...
                T.blockIndex(i,2)+blockCols,:);
   L1 = T.L(T.blockIndex(i,1)+blockRows,...
            T.blockIndex(i,2)+blockCols,:);
   M = T.mask(T.blockIndex(i,1)+blockRows,...
              T.blockIndex(i,2)+blockCols,:);
   
   % Find a similar block in the source texture.
   % Note: Either use pre-cached blocks or draw from input texture.
   B2 = transfer(S,B1,L1,M,alpha,errorTol);
   
   % Evaluate minimum boundary cut.
   if bndCut
      [B3,C] = mincut(B1,B2,M);
      T.cut(T.blockIndex(i,1)+blockRows,...
            T.blockIndex(i,2)+blockCols,:) = ...
         T.cut(T.blockIndex(i,1)+blockRows,...
               T.blockIndex(i,2)+blockCols,:) | C;
   else
      B3 = B2;
   end
   
   % Update output/mask values for this block.
   T.image(T.blockIndex(i,1)+blockRows,...
           T.blockIndex(i,2)+blockCols,:) = B3;
   T.mask(T.blockIndex(i,1)+blockRows,...
          T.blockIndex(i,2)+blockCols,:) = 1;
       
   % Display synthesis progress.
   figure(2); clf;
   imagesc(T.image);
   axis image; drawnow;
  
end % End of IQ synthesis.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Display results.

% Display synthesized texture.
figure(2); clf;
imagesc(T.image);
axis image; set(gcf,'Name','Synthesized Texture');

% Display synthesized texture with boundary cuts.
if bndCut
   figure(3); clf;
   I = T.image;
   R = I(:,:,1); G = I(:,:,2); B = I(:,:,3);
   idx = find(T.cut); R(idx) = 1*T.cut(idx) + 0*R(idx);
   G(idx) = 0; B(idx) = 0;
   I = cat(3,R,G,B); imagesc(I); axis image;
   axis image; set(gcf,'Name','Synthesized Texture and Boundary Cuts');
end