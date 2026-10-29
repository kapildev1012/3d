function final_codes_validate(cases_file, out_file, code_dir)
% Run forward_tensegrity.m from code_dir (default: the ORIGINAL final_codes/original)
% on the cases written by tools/final_codes_validate.mjs and save the results as JSON.
% A failed solve stops the original with an error (nodal_positions is not assigned);
% it is caught here and recorded as output = 0.
here = fileparts(mfilename('fullpath'));
if nargin < 3, code_dir = fullfile(here, '..', 'final_codes', 'original'); end
addpath(code_dir);
if exist('OCTAVE_VERSION', 'builtin'), addpath(fullfile(here, 'octave_shims')); end
c = jsondecode(fileread(cases_file));
n = size(c.theta, 1);
output = zeros(n, 1); positions = nan(n, 18); seconds = zeros(n, 1);
for i = 1:n
  t0 = tic;
  try
    [o, P] = forward_tensegrity(c.theta(i, :), c.force(i, :));
    output(i) = o;
    if o == 1, positions(i, :) = reshape(P, 1, 18); end
  catch
    output(i) = 0;
  end
  seconds(i) = toc(t0);
end
fid = fopen(out_file, 'w');
list = @(v, fmt) ['[' strjoin(arrayfun(@(x) sprintf(fmt, x), v(:)', 'UniformOutput', false), ',') ']'];
fprintf(fid, '{"output":%s,"seconds":%s,"positions":[', list(output, '%d'), list(seconds, '%.6g'));
for i = 1:n
  row = sprintf('%.17g,', positions(i, :)); row = strrep(row(1:end-1), 'NaN', 'null');
  fprintf(fid, '[%s]', row);
  if i < n, fprintf(fid, ','); end
end
fprintf(fid, ']}');
fclose(fid);
end
