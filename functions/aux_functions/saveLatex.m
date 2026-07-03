function saveLatex(fig_handle,name,opt)
assert(isfield(opt,'file_path'))
file_path = opt.file_path;
% Ex: file_path = 'C:\NoBackup\Svk_Git\rotorvinkel\rotorvinkel-artiklar\V_ctrl_coord\figures\';

% save_fig true/false
if isfield(opt,'save_fig')
    save_fig = opt.save_fig;
else
    save_fig = true;
end

if isfield(opt,'file_format')
    file_format = opt.file_format;
else
    file_format = 'pdf';
end

if strcmp(file_format,'eps')
    file_format = 'epsc'; % The user probably meant to save as epsc
end

if strcmp(file_format,'epsc')
    if length(name)> 4
        name_format = name(end-3:end);
        if strcmp(name_format,'.eps')
            name = [name,'.eps'];
        end
    else
        name = [name,'.eps'];
    end
end

if strcmp(file_format,'pdf')
    name = [name,'.pdf'];
end

%Image File Formats

% Option	Format	                            Default File Extension
% 'jpeg'	JPEG 24-bit	                        .jpg
% 'png'	    PNG 24-bit	                        .png
% 'tiff'	TIFF 24-bit (compressed)	        .tif
% 'tiffn'	TIFF 24-bit (not compressed)        .tif
% 'meta'	Enhanced metafile (Windows only)    .emf

% Vector Graphics Formats
% 
% Option	Format	                                                Default File Extension
% 'pdf'	    Full page Portable Document Format (PDF) color	        .pdf
% 'eps'	    Encapsulated PostScript® (EPS) Level 3 black and white	.eps
% 'epsc'	Encapsulated PostScript (EPS) Level 3 color	            .eps
% 'eps2'	Encapsulated PostScript (EPS) Level 2 black and white	.eps
% 'epsc2'	Encapsulated PostScript (EPS) Level 2 color	            .eps
% 'meta'	Enhanced Metafile (Windows® only)	                    .emf
% 'svg'	    SVG (scalable vector graphics)	                        .svg

if save_fig
    if strcmp(file_format,'pdf')
        exportgraphics(fig_handle,[file_path,name],'ContentType','vector')
    else
        saveas(fig_handle,[file_path,name],file_format)
    end
    name = [name, ' (saved to path)'];
end

set(fig_handle,'name',name)

end

