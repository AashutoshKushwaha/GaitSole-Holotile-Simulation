% camargo_table_to_csv.m
% Converts Camargo MATLAB v7 *table* .mat files to plain CSVs that Python
% can read. Run this on MATLAB Online (real MATLAB reads table objects
% natively; scipy/pymatreader cannot).
%
% HOW TO USE (MATLAB Online, https://matlab.mathworks.com):
%   1. Upload  camargo_convert.zip  to your MATLAB Drive (drag-and-drop).
%   2. Upload (or paste into a new file) this script next to it.
%   3. Run it:   >> camargo_table_to_csv
%   4. Download the produced  camargo_csv.zip  back to your PC.
%
% It auto-unzips camargo_convert.zip if the folder isn't already extracted.

clear; clc;

ZIP_IN  = 'camargo_convert.zip';
SRC_DIR = 'camargo_convert';
OUT_DIR = 'csv_out';
ZIP_OUT = 'camargo_csv.zip';

% --- unzip input if needed ---
if ~isfolder(SRC_DIR)
    if isfile(ZIP_IN)
        fprintf('Unzipping %s ...\n', ZIP_IN);
        unzip(ZIP_IN, SRC_DIR);
    else
        error('Neither folder "%s" nor "%s" found in current directory.', SRC_DIR, ZIP_IN);
    end
end

if isfolder(OUT_DIR); rmdir(OUT_DIR, 's'); end
mkdir(OUT_DIR);

% --- find every .mat recursively ---
files = dir(fullfile(SRC_DIR, '**', '*.mat'));
fprintf('Found %d .mat files.\n', numel(files));

nok = 0;
for k = 1:numel(files)
    fpath = fullfile(files(k).folder, files(k).name);
    s = load(fpath);
    fn = fieldnames(s);

    % the saved variable (the table) is the first/only field
    v = s.(fn{1});

    [~, bname] = fileparts(files(k).name);
    [~, parent] = fileparts(files(k).folder);   % 'ik', 'fp', or 'camargo_convert'
    outname = fullfile(OUT_DIR, sprintf('%s__%s.csv', parent, bname));

    if istable(v)
        writetable(v, outname);
        fprintf('  OK   %-28s -> %s  (%d rows x %d cols)\n', files(k).name, outname, height(v), width(v));
        nok = nok + 1;
    elseif isstruct(v) && numel(fieldnames(v)) >= 1
        % SubjectInfo may come back as a struct of columns -> coerce to table
        try
            T = struct2table(v);
            writetable(T, outname);
            fprintf('  OK*  %-28s -> %s  (struct->table)\n', files(k).name, outname);
            nok = nok + 1;
        catch ME
            fprintf('  SKIP %-28s : struct not convertible (%s)\n', files(k).name, ME.message);
        end
    else
        fprintf('  SKIP %-28s : class %s (not a table)\n', files(k).name, class(v));
    end
end

fprintf('\nConverted %d files. Zipping -> %s\n', nok, ZIP_OUT);
zip(ZIP_OUT, OUT_DIR);
fprintf('DONE. Download "%s" and send it back.\n', ZIP_OUT);
