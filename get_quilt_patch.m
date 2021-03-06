function [ out_im ] = get_quilt_patch(patch, Im, patch_size, overlap_size, i1, j1)
    [r, c, ~] = size(Im);
    out_im = patch;  % Default case
    
    if ((i1 == 1) && (j1 == 1))
        % Pick the first patch randomly
        r_take = randi([1, r - patch_size(1)], 1, 1);
        c_take = randi([1, c - patch_size(2)], 1, 1);
        out_im = Im(r_take:r_take+patch_size(1)-1, c_take:c_take+patch_size(2)-1, :);
    elseif (i1 == 1)
        % top row, consider only the left overlap
        if (patch_size(2) >= overlap_size(2) && overlap_size(2) >= 3)
            left_overlap = patch(:, 1:overlap_size(2), :);
            out_im = quilt_left_overlap(left_overlap, Im, patch_size, 200);
        end
    elseif (j1 == 1)
        % left column, consider only the top overlap
        if (patch_size(1) >= overlap_size(1) && overlap_size(1) >= 3)
            top_overlap = patch(1:overlap_size(1), :, :);
            out_im = quilt_top_overlap(top_overlap, Im, patch_size, 200);
        end
    else
        % In the interior, consider the L-overlap
        if (patch_size(1) >= overlap_size(1) && overlap_size(1) >= 3 && patch_size(2) >= overlap_size(2) && overlap_size(2) >= 3)
            top_overlap = patch(1:overlap_size(1), :, :);
            left_overlap = patch(:, 1:overlap_size(2), :);
            out_im = quilt_L_overlap(top_overlap, left_overlap, Im, patch_size, 200);
        end
    end
end
