function [k, numItems] = txtmenu(varargin)
%TXTMENU Generate an ASCII text list menu in the command window.
%   CHOICE = TXTMENU('HEADER', DESC0, DESC1, DESC2, ..., DESCn) displays a
%   list of possible choices and descriptions,
%
%       ----- HEADER -----
%
%          >> 0) DESC0
%             1) DESC1
%             2) DESC2
%             ...  ...
%             n) DESCn
%
%       Select a menu item:
%
%   and the user is prompted to make a selection, which is returned as
%   CHOICE. No selection (i.e., hitting the return key without an input)
%   returns the default option as indicated by the >>.
%
%   CHOICE = TXTMENU(HEADER, DESCLIST) where DESCLIST is a cell array of
%   the form {DESC0, DESC1, DESC2, ..., DESCn} is also valid.
%
%   CHOICE = TXTMENU(HEADER, DEFAULT, ... ) sets the default to the integer
%   DEFAULT instead of 0. For no default, use an empty matrix ([]).
%
%   CHOICE = TXTMENU(MENUSTRUCT) allows for more control over the menu when
%   MENUSTRUCT is a struct with some or all of the following fields (field
%   names are not case sensitive):
%       DESCLIST   - Cell array of the form {DESC1, DESC2,..., DESCn}. Only
%                    mandatory field.
%       HEADER     - HEADER string. Default is 'MENU'.
%       PROMPT     - PROMPT string. Default is 'Select a menu item:'.
%       CHOICELIST - Vector containing numeric choices or cell array or
%                    choices. Combining numeric and string choices is
%                    allowable. Default is 0:nItems-1.
%       DEFAULT    - Value returned for no selection (i.e. hitting return
%                    key without an input). Must match an element, numeric
%                    or char, in CHOICELIST. Default is 0 only if no
%                    CHOICELIST is provided. Otherwise, DEFAULT = '',
%                    signifying no default option.
%     Fields for formatting the appearance of the menu:
%       HEADERLEADER  - String preceding the header.
%       HEADERTRAILER - String succeeding the header.
%       LEFTSTR       - String preceding each choice.
%       DEFLEFTSTR    - String preceding default choice.
%       RIGHTSTR      - String separating each choice and its description.
%       DEFRIGHTSTR   - String separating default choice and default
%                       description. Should be the same length as RIGHTSTR.
%
%   The line will be omitted and the choice made illegal for description
%   items that are empty strings (''), even if the line is the default.
%   Similarly, if the HEADER is empty, the entire header line is omitted.
%
%   HEADER may also be a cell array, where the first string is still the
%   header row and the second string in the cell array replaces the default
%   prompt 'Select a menu item:'.
%
%   TXTMENU will return ONLY values for CHOICE which appear on the options
%   list.
%
%   TXTMENU uses string matching, so an input of 1.0 will not match a menu
%   item 1.
%
%   Example 1: Choose a color; default to red.
%       K = TXTMENU('Choose a color','Red','Blue','Green')
%
%   Example 2: Choose a color; force a choice.
%       K = TXTMENU('Choose a color',[],'Red','Blue','Green')
%
%   Example 3: Choose a color; force a choice; number as 2,4,6.
%       K = TXTMENU('Choose a color',{'' '' 'Red' '' 'Blue' '' 'Green'})
% 
%   Example 4: Choose a number; use alternate prompt; default to 7.
%       K = TXTMENU({'' 'Choose a prime:'},7,...
%           '','',' ',' ','',' ','',' ','','','',' ','',' ','','','',' ')
%
%   Example 5: Choose a color; return corresponding character.
%       S = struct('descList',{{sprintf('N/A\n') 'Red' 'Blue' 'Green'}},...
%           'choiceList',{{0 'r' 'b' 'g'}}, 'default','b',...
%           'prompt','Please choose a color:',...
%           'defLeftStr','<','defRightStr','> ','rightStr','  ');
%       K = TXTMENU(S)
%
%   See also MENU, INPUT, SPRINTF.

%   Copyright 2010-2013 Sky Sartorius
%   Author contact: mathworks.com/matlabcentral/fileexchange/authors/101715


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%            User-definable default list appearance options             %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Use these options to customized how the list columns will be separated
% and how to signify which is the default option. Remember that the length
% of rightStr and defRightStr should be the same. These appearance options
% can also be set progammatically by providing a struct with the
% appropriate fields.

headerLeader = '----- '; headerTrailer = ' -----';

% Length for right strings is 1:
% leftStr    = '';      rightStr    = ' ';
% defLeftStr = '>> ';   defRightStr = ' ';
% defLeftStr = '•';     defRightStr = ' ';

% Length for right strings is 2:
leftStr    = '';      rightStr    = ') ';
defLeftStr = '>> ';   defRightStr = ') ';
% defLeftStr = '•';     defRightStr = ') ';
% leftStr    = '';      rightStr    = '  ';
% defLeftStr = '<';     defRightStr = '> ';

% Length for right strings is 3:
% leftStr    = '';      rightStr    = ' - ';
% defLeftStr = '>> ';   defRightStr = ' - ';
% defLeftStr = 'return/'; defRightStr = ' - ';
% defLeftStr = '';      defRightStr = ' • ';

% Length for right strings is 4:
% leftStr    = '';      rightStr    = '  - ';
% defLeftStr = '«';     defRightStr = '» - ';
% defLeftStr = '(';     defRightStr = ') - ';
% defLeftStr = '<';     defRightStr = '> - ';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Figure out the inputs.
narginchk(1,inf);

% Set default parameters.
k = nan;
prompt = 'Select a menu item:';
default = '0';

if nargin == 1 && isstruct(varargin{1}) % Use struct input scheme
    S = varargin{1};
    % Set all struct fields to lowercase
    S = cell2struct(struct2cell(S),lower(fieldnames(S)),1);
    
    % Get menu parameters, if present
    if isfield(S,'desclist')
        itemList = S.desclist;
        numItems = length(itemList);
    else
        error('DESCLIST is required.') % Only mandatory field
    end
    if isfield(S,'header')
        header = S.header;
    else
        header = 'MENU';
    end
    if isfield(S,'prompt')
        prompt = S.prompt;
        % default is set prior.
    end
    if isfield(S,'choicelist')
        choiceList = S.choicelist;
        % Check that length is appropriate.
        if length(choiceList) ~= numItems;
            % Also note that the numItems output isn't documented because I
            % can't think of a use case.
            error('CHOICELIST and DESCLIST must be of equal length.')
        end
    else
        choiceList = 0:numItems-1;
    end
    if isfield(S,'default')
        default = S.default;
        % Make sure default is scalar or string
        validNumber = isscalar(default);
        validString = ischar(default) && isvector(default);
        if ~validNumber && ~validString
            error('Default choice must be scalar or string.')
        end
    elseif isfield(S,'choicelist')
        % Must specify default if specifying choiceList.
        default = [];
    end
    
    if isfield(S,'defleftstr')
        defLeftStr = S.defleftstr;
    end
    if isfield(S,'defrightstr')
        defRightStr = S.defrightstr;
    end
    
    if isfield(S,'leftstr')
        leftStr = S.leftstr;
    end
    if isfield(S,'rightstr')
        rightStr = S.rightstr;
    end
    
    if isfield(S,'headerleader')
        headerLeader = S.headerleader;
    end
    if isfield(S,'headertrailer')
        headerTrailer = S.headertrailer;
    end
    % TODO: find out if there's a more elegant way to do this that the many
    % "if isfield..." blocks.
    
else % Use normal input scheme
    if nargin < 2
        warning('No menu items to choose from.')
        return;
    end
    
    header = varargin{1};
    
    % If second argument is DEFAULT.
    if (nargin > 2) && isnumeric(varargin{2}) && (numel(varargin{2}) <= 1)
        % We check later to make sure it's an element of the choiceList,
        % also ensuring it's integer.
        default = varargin{2}; % num2str(default) comes later.
        itemList = varargin(3:end);
    else
        itemList = varargin(2:end);
    end
    
    % Differentiate between list input and cell array input.
    if (length(itemList) == 1) && iscell(itemList{1});
        itemList = itemList{1};
    end
    
    % If default not provided, check on 0 option
    if strcmp(default,'0') && ... % No default provided
            isempty(itemList{1}) % But 0 line omitted from list
        default = [];
    end
    
    numItems = length(itemList);
    choiceList = 0:numItems-1;
end

%% A little more input processing
% Get rid of elements of itemList and choiceList where description is ''.
emptyInd = cellfun('isempty',itemList);
choiceList(emptyInd) = [];
itemList(emptyInd) = [];
numItems = length(itemList);

% Case where header contains header and prompt:
if iscell(header) && length(header) == 2
    prompt = header{2};
    header = header{1};
end

%% Do some more checks.
% itemList must be a cell vector
if ~iscell(itemList)
    error('Invalid description list.')
end
if ~isvector(itemList)
    error('Description list must be cell vector.')
end
% Item list is a cell vector, but not necessarily of strings. Fix that:
itemList = cellfun(@num2str,itemList,'UniformOutput',false);

% If for some reason header isn't a string...
if ~isempty(header)
    if ~ischar(header)
        error('Header must a string (or empty)')
    end
end

% Check that have something to work with
if numItems == 0;
    warning('Incomplete list.')
    k=NaN;
    return
end


%% Process choice list into unique cell vector of strings.
% Must save original form of list for returning final value.
choiceListOrig = choiceList;
if isnumeric(choiceList)
    choiceList = num2cell(choiceList);
elseif ~isvector(choiceList)
    error('Choice list must be a vector.')
elseif ~iscell(choiceList)
    error('Choice list must be numeric or cell vector.')
end
choiceList = cellfun(@num2str,choiceList,'UniformOutput',false);
% Must preserve list before appending and prepending stuff in order to
% check matches for inputs
choiceListStr = choiceList;
if ~isequal(choiceList,unique(choiceList,'stable'))
    error('Choice list must have only unique entries.')
end

%% Check to make sure default appears exactly once in choiceList.
default = num2str(default);
if isempty(default)
    haveDefault = false;
    iDefault = false(numItems,1);
    nDefault = [];
else
    haveDefault = true;
    iDefault = cellfun(@(c) isequal(c,default),choiceList);
    if 1 ~= nnz(iDefault);
        error('Default must appear exactly once in list of options.')
    end
    nDefault = find(iDefault);
end


%% Process choicelist into something pretty w/ formatting
% This step must come after finding where the default is.

% Put parentheses or whatever symbols around choices.
choiceList(~iDefault) = cellfun(@(c) [leftStr c rightStr],...
    choiceList(~iDefault),'UniformOutput', false);
if haveDefault
    choiceList{iDefault} = [defLeftStr choiceList{iDefault} defRightStr];
end
% Make all choices same length so they line up.
choiceLength = max(8,max(cellfun(@length,choiceList)));
choiceList = cellfun(@(s) sprintf(...
    [' %' num2str(choiceLength) 's'],s),choiceList,...
    'UniformOutput',false);


while 1
    %% Display menu.
    disp(' ')
    if ~isempty(header)
        disp([headerLeader header headerTrailer])
    end
    disp(' ')
    
    for i = 1 : numItems
        disp([choiceList{i} itemList{i}])
    end
    disp(' ')
    k = input([prompt ' '],'s');
    
    %% Evaluate input.
    % If empty, do something with the default.
    if isempty(k) % Default option selected.
        kInd = nDefault; % Could be empty.
    else
        % Otherwise, look for a string match with the version of the choice
        % list before prepending and appending stuff.
        kInd = find(strcmp(k,choiceListStr));
    end
    
    % There can be a case where str2num(k) is a match for something in
    % choiceListOrig. However, this case is pretty obscure and usually
    % would requre more prompting, so I'm not accounting for it, though it
    % is in the help block that it does string matches.
    
    if kInd % Have a match. Yay!
        % NB: Won't get to here for empty kInd
        if iscell(choiceListOrig) % Original can be cell
            k = choiceListOrig{kInd};
        else % or numeric
            k = choiceListOrig(kInd);
        end
        return
    end
    
    % No success.
    beep
    disp(' ')
    disp('Selection out of range. Try again.')
end


% Revision history:
%{
V1.0    24 July 2010
V2.0    26 July 2010
  Allowed for the use of \n, \t, etc. in input strings
  Put prompt as first active line so it can be easily changed, but
  decided not to have it be a second input for reasons of
  compatibility with previous versions as well as MENU, and also
  because of complicating use.
V3.0    4 August 2010
  Moved default option to beginning of list. The numerical sequence
  is maintained this way, and since the default value will often be
  known very early in development, it saves more changing of
  following code.
  Allow empty header and omission of header line
V3.1    15 February 2011
  You can now control the input prompt by using a 2 cell array for
  the first input
2013-04-17
  got rid of unnecessary sprintf and strcmp(class(...
2013-10-18 - 21
    total rework, now including struct input, much more input checking, and
    many more features.
    help block updates and more examples
%}
