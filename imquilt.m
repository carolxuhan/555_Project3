
% Reset Matlab environment.

% Set Image Quilting (IQ) parameters.
sourceTexture = './textures/examples/synthetic.jpg'; % source texture
blockSize     = [15 15];              % block size
blockOverlap  = ceil(blockSize/5);    % block overlap
scaleFactor   = 2;                    % scale factor for synthesized texture
preCache      = false;                % enable/disable source block caching
errorTol      = 0.1;                  % matching-error tolerance
colorSpace    = 'RGB';                % select either RGB or L*A*B colors
bndCut        = true;                 % enable/disable boundary optimization

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Pre-process source texture (to extract individual blocks for synthesis).

% Load source texture.
S.image    = im2double(imread(sourceTexture));
S.nrows    = size(S.image,1);
S.ncols    = size(S.image,2);
S.ncolors  = size(S.image,3);
S.nblocks  = [S.nrows S.ncols]-blockSize+1;

% Convert to L*A*B* color space (if requested).
if strcmp(colorSpace,'LAB')
   S.image = applycform(S.image,makecform('srgb2lab'));
end

% Display source texture.
figure(1); clf; 
if strcmp(colorSpace,'RGB')
   imagesc(S.image);
else
   imagesc(applycform(S.image,makecform('lab2srgb')));
end
set(gcf,'Name','Source Texture');
if S.ncolors == 1
   colormap gray;
end
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

% Concatenate source blocks into a single matrix.
if preCache
   S.blocks = zeros(S.ncolors*prod(blockSize),prod(S.nblocks));
   for i = 1:S.nblocks(1)
      for j =1:S.nblocks(2)
         S.blocks(:,(i-1)*S.nblocks(2)+j) = ...
            reshape(S.image(i+blockRows,j+blockCols,:),[],1);
      end
   end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Apply Image Quilting (IQ) to generate a synthetic texture.

% Allocate storage for the synthesized output image/mask.
% Note: Increase size to prevent clipped blocks at edges.
%       Remember to crop output image after synthesis.
overlapSize = blockSize-blockOverlap;
T.nrows   = overlapSize(1)*(ceil(scaleFactor*S.nrows/overlapSize(1))-1)+blockSize(1);
T.ncols   = overlapSize(2)*(ceil(scaleFactor*S.ncols/overlapSize(2))-1)+blockSize(2);
T.ncolors = S.ncolors;
T.nblocks = floor([T.nrows T.ncols]./overlapSize);
T.image   = zeros(T.nrows,T.ncols,T.ncolors);
T.mask    = false(T.nrows,T.ncols,T.ncolors);
T.cut     = false(T.nrows,T.ncols);

% Determine locations of overlapping output blocks.
[C,R] = meshgrid(overlapSize(2)*(0:T.nblocks(2)-1)+1,...
                 overlapSize(1)*(0:T.nblocks(1)-1)+1);
R = R'; C = C'; T.blockIndex = [R(:) C(:)];

% Seed the synthesized texture with a random block.
i = ceil(prod(S.nblocks)*rand);
B = S.image(S.blockIndex(i,1)+blockRows,...
            S.blockIndex(i,2)+blockCols,:);
T.image(blockRows+1,blockCols+1,:) = B;
T.mask(blockRows+1,blockCols+1,:) = 1;

% Display synthesized texture.
figure(2); clf;
if strcmp(colorSpace,'RGB')
   imagesc(T.image);
else
   imagesc(applycform(T.image,makecform('lab2srgb')));
end
set(gcf,'Name','Synthesized Texture');
if S.ncolors == 1
   colormap gray;
end
axis image; drawnow;

% Synthesize the remaining pixels using IQ.
for i = 2:prod(T.nblocks)

   % Extract current value (i.e., the overlapping region).
   B1 = T.image(T.blockIndex(i,1)+blockRows,...
                T.blockIndex(i,2)+blockCols,:);
   M = T.mask(T.blockIndex(i,1)+blockRows,...
              T.blockIndex(i,2)+blockCols,:);
   
   % Find a similar block in the source texture.
   % Note: Either use pre-cached blocks or draw from input texture.
   B2 = findmatch(S,B1,M,preCache,errorTol);
   
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
   if strcmp(colorSpace,'RGB')
      imagesc(T.image);
   else
      imagesc(applycform(T.image,makecform('lab2srgb')));
   end
   axis image; drawnow;
  
end % End of IQ synthesis.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Display results.

% Display synthesized texture.
figure(2); clf;
if strcmp(colorSpace,'RGB')
   imagesc(T.image);
else
   imagesc(applycform(T.image,makecform('lab2srgb')));
end
axis image; set(gcf,'Name','Synthesized Texture');
if S.ncolors == 1
   colormap gray;
end

% Display synthesized texture with boundary cuts.
if bndCut
   figure(3); clf;
   if strcmp(colorSpace,'RGB')
      I = T.image;
   else
      I = applycform(T.image,makecform('lab2srgb'));
   end
   if S.ncolors ==1
      I = repmat(I,[1 1 3]);
   end
   R = I(:,:,1); G = I(:,:,2); B = I(:,:,3);
   idx = find(T.cut); R(idx) = 1*T.cut(idx) + 0*R(idx);
   G(idx) = 0; B(idx) = 0;
   I = cat(3,R,G,B); imagesc(I); axis image;
   axis image; set(gcf,'Name','Synthesized Texture and Boundary Cuts');
end