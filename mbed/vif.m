function out = vif(truth,yes,no)
% VIF(TRUTH, YES, NO) is a function interface for simple vectorised if-else
% statements.  It returns a value with the same shape as TRUTH, with values
% taken from YES or NO, depending upon whether the corresponding element of
% TRUTH is true or false. YES and NO must be scalars or matrices with the
% same shape as TRUTH.

% Example:
% vif(eye(3), 4, 1+1i)
% ans =
% 
%    4.0000             1.0000 + 1.0000i   1.0000 + 1.0000i
%    1.0000 + 1.0000i   4.0000             1.0000 + 1.0000i
%    1.0000 + 1.0000i   1.0000 + 1.0000i   4.0000
% 
% vif(eye(3), rand(3), magic(3))
% ans =
% 
%     0.4218    1.0000    6.0000
%     3.0000    0.6557    7.0000
%     4.0000    9.0000    0.6787
% 
% vif is often faster than looped if-else
% tic; for i=1:1000000; if 1; 4; else 1+i; end; end; toc
% Elapsed time is 0.831261 seconds.
% 
% truth = rand(1000000,1)< 0.5;
% tic; vif(truth, 4, 1+i); toc
% Elapsed time is 0.097463 seconds.
% 
% ...but slower for the all-scalar case
% tic; for(i=1:100000); vif(1, 4, 1+i); end; toc
% Elapsed time is 12.750187 seconds.
% 
% $ Author: Richard Cotton $		$ Date: 2008/10/01 $    $ Version 1.4 $

% Basic error checking of inputs
if nargin ~= 3
    error('vif:wrongNumberOfInputs', 'This function requires 3 inputs.');
end

% validateattributes gives nice error messages but is slow.
% validateattributes(truth, 'logical', {}, 'vif', 'Truth')
% oktypes = {'numeric', 'logical', 'char'};
% validateattributes(yes, oktypes, {}, 'vif', 'Yes')
% validateattributes(no, oktypes, {}, 'vif', 'No')

function checktypes(x)   
   if ~isnumeric(x) && ~islogical(x) && ~ischar(x)
      error('vif:badType', 'All inputs must be numeric, logical or char.');
   end
end
checktypes(truth);
checktypes(yes);
checktypes(no);

isscalaryes = isscalar(yes);
isscalarno = isscalar(no);
sizetruth = size(truth);

if (~isscalaryes && ~isequal(size(yes), sizetruth)) || ...
		(~isscalarno && ~isequal(size(no), sizetruth)) 
	error('vif:badYesNoValue', ...
      'Yes and No must be scalars or the same size as Truth.');
end

% Coerce real numeric 'truth' to logical if necessary
if isreal(truth)
   truth = ~~truth;	% quicker than logical(truth) for large matrices
else
   truth = ~~real(truth);
   warning('vif:complexTruth', ...
      'Truth is complex - only the real part will be used.');   
end

if ~ischar(yes) && ~ischar(no) && ...
      all(isfinite(yes(:))) && all(isfinite(no(:)))
   out = truth.*yes + ~truth.*no;
else
   if isscalaryes
      out = yes*ones(sizetruth);
   else
      out = yes;
   end
   
   if isscalarno
      out(~truth) = no;
   else
      out(~truth) = no(~truth);
   end
end
end