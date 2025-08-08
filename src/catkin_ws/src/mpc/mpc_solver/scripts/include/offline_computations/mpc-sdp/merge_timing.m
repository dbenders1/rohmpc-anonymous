function [timing] = merge_timing(timing,timing_to_merge)
  fields = fieldnames(timing_to_merge);
  for i=1:numel(fields)
    timing.(fields{i}) = timing_to_merge.(fields{i});
  end
end
