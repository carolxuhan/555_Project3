%function Y = imagequilt(X, tilesize, n, overlap, err)
%Performs the Efros/Freeman Image quilting algorithm on the input
%
%Inputs
%   X:  The source image to be used in synthesis
%   tilesize:   the dimensions of each square tile.  Should divide size(X) evenly
%   n:  The number of tiles to be placed in the output image, in each dimension
%   overlap: The amount of overlap to allow between pixels (def: 1/6 tilesize)
%   err: used when computing list of compatible tiles (def: 0.1)

function Y = imagequilt(X, tilesize, n, overlap, err)

X = double(X);

if( length(size(X)) == 2 )
    X = repmat(X, [1 1 3]);
elseif( length(size(X)) ~= 3 )
    error('Input image must be 2 or 3 dimensional');
end;
    
simple = 1Z;

if( nargin < 5 )
    err = 0.002;
end;

if( nargin < 4 )
    overlap = round(tilesize / 6);
end;

% if( size(X,1) ~= size(X,2) )
%     error('Must be square');
% end;

if( overlap >= tilesize )
    error('Overlap must be less than tilesize');
end;

destsize = n * tilesize - (n-1) * overlap 

Y = zeros(destsize, destsize, 3);

for i=1:n,
     for j=1:n,
         startI = (i-1)*tilesize - (i-1) * overlap + 1;
         startJ = (j-1)*tilesize - (j-1) * overlap + 1;
         endI = startI + tilesize -1 ;
         endJ = startJ + tilesize -1;
         
         %Determine the distances from each tile to the overlap region
         %This will eventually be replaced with convolutions
         distances = zeros( size(X,1)-tilesize, size(X,2)-tilesize );         

%          K = ones(tilesize, overlap);
%          y = Y(startI:endI,startJ:endJ,1:3);
%          for k=1:3,
%                 
%          end;
         
         
%          M = zeros(tilesize, tilesize);
%          M(1:overlap,:) = 1;
%          M(:,1:overlap) = 1;
%          
%          y = Y(startI:endI,startJ:endJ,1:3);
%          a = (y(:,:,1) .* M) + (y(:,:,2) .* M) + (y(:,:,3) .* M);
%          a2 = sum(sum(sum(a.^2)));
%             
%          %a2 = sum(sum(sum(a.^2)));
%          
%          b2 = filter2(M, b.^2);
%          
%          ab = filter2(a, b);
%      
%          distances = sqrt(a2 + (2*ab + b2));
%          distances = distances(1:size(X,1)-tilesize, 1:size(X,2)-tilesize);
         
%          b2 = filter2(X, zerosones(tilesize, tilesize)
%          
%          a2 = sum(sum(Y(startI:endI, startJ:startJ+overlap-1, 1:3).^2)) + ...
%               sum(sum(Y(startI:startI+overlap-1,startJ+overlap:endJ, 1:3).^2));
%          b2 = sum(sum(
%          

        useconv = 1;
        
        if( useconv == 0 )
            
            %Compute the distances from the template to target for all i,j
            for a = 1:size(distances,1)
                v1 = Y(startI:endI, startJ:endJ, 1:3);
                for b = 1:size(distances,2),                 
                    v2 = X(a:a+tilesize-1,b:b+tilesize-1, 1:3);
                    distances(a,b) = myssd( double((v1(:) > 0)) .* (v1(:) - v2(:)) );
                    %distances(a,b) = D;    
                end;
            end;
            
        else
            
            %Compute the distances from the source to the left overlap region
            if( j > 1 )
                distances = ssd( X, Y(startI:endI, startJ:startJ+overlap-1, 1:3) );    
                distances = distances(1:end, 1:end-tilesize+overlap);
            end;
            
            %Compute the distance from the source to top overlap region
            if( i > 1 )
                Z = ssd( X, Y(startI:startI+overlap-1, startJ:endJ, 1:3) );
                Z = Z(1:end-tilesize+overlap, 1:end);
                if( j > 1 ) distances = distances + Z;
                else distances = Z;
                end;
            end;
            
            %If both are greater, compute the distance of the overlap
            if( i > 1 && j > 1 )
                Z = ssd( X, Y(startI:startI+overlap-1, startJ:startJ+overlap-1, 1:3) );
                Z = Z(1:end-tilesize+overlap, 1:end-tilesize+overlap);                   
                distances = distances - Z;
            end;
            
            %distances = distances(1:end-tilesize, 1:end-tilesize);
            
        end;

         %Find the best candidates for the match
         best = min(distances(:));
         candidates = find(distances(:) <= (1+err)*best);
          
         idx = candidates(ceil(rand(1)*length(candidates)));
                         
         [sub(1), sub(2)] = ind2sub(size(distances), idx);
         fprintf( 'Picked tile (%d, %d) out of %d candidates.  Best error=%.4f\n', sub(1), sub(2), length(candidates), best );       
         
         %If we do the simple quilting (no cut), just copy image
         if( simple )
             Y(startI:endI, startJ:endJ, 1:3) = X(sub(1):sub(1)+tilesize-1, sub(2):sub(2)+tilesize-1, 1:3);
         else
             
             %Initialize the mask to all ones
             M = ones(tilesize, tilesize);
             
             %We have a left overlap
             if( j > 1 )
                 
                 %Compute the SSD in the border region
                 E = ( X(sub(1):sub(1)+tilesize-1, sub(2):sub(2)+overlap-1) - Y(startI:endI, startJ:startJ+overlap-1) ).^2;
                 
                 %Compute the mincut array
                 C = mincut(E, 0);
                 
                 %Compute the mask and write to the destination
                 M(1:end, 1:overlap) = double(C >= 0);
                 %Y(startI:endI, startJ:endJ, :) = filtered_write(Y(startI:endI, startJ:endJ, :), ...
                 %    X(sub(1):sub(1)+tilesize-1, sub(2):sub(2)+tilesize-1, :), M); 
                 
                 %Y(startI:endI, startJ:endJ, 1:3) = X(sub(1):sub(1)+tilesize-1, sub(2):sub(2)+tilesize-1, 1:3);
                 
                 %Compute the mask and write to the destination
                 %                  M = zeros(tilesize, tilesize);
                 %                  M(1:end, 1:overlap) = double(C == 0);
                 %                  Y(startI:endI, startJ:endJ, :) = filtered_write(Y(startI:endI, startJ:endJ, :), ...
                 %                      repmat(255, [tilesize, tilesize, 3]), M); 
                 
             end;
             
             %We have a top overlap
             if( i > 1 )
                 %Compute the SSD in the border region
                 E = ( X(sub(1):sub(1)+overlap-1, sub(2):sub(2)+tilesize-1) - Y(startI:startI+overlap-1, startJ:endJ) ).^2;
                 
                 %Compute the mincut array
                 C = mincut(E, 1);
                 
                 %Compute the mask and write to the destination
                 M(1:overlap, 1:end) = M(1:overlap, 1:end) .* double(C >= 0);
                 %Y(startI:endI, startJ:endJ, :) = filtered_write(Y(startI:endI, startJ:endJ, :), ...
                 %    X(sub(1):sub(1)+tilesize-1, sub(2):sub(2)+tilesize-1, :), M); 
             end;
             
             
             if( i == 1 && j == 1 )
                 Y(startI:endI, startJ:endJ, 1:3) = X(sub(1):sub(1)+tilesize-1, sub(2):sub(2)+tilesize-1, 1:3);
             else
                 %Write to the destination using the mask
                 Y(startI:endI, startJ:endJ, :) = filtered_write(Y(startI:endI, startJ:endJ, :), ...
                     X(sub(1):sub(1)+tilesize-1, sub(2):sub(2)+tilesize-1, :), M); 
             end;
             
         end;
                 
             
         image(uint8(Y));
         drawnow;
     end;
end;

figure;
image(uint8(Y));

function y = myssd( x )
y = sum( x.^2 );

function A = filtered_write(A, B, M)
for i = 1:3,
    A(:, :, i) = A(:,:,i) .* (M == 0) + B(:,:,i) .* (M == 1);
end;
