import os
import re
import codecs

class SourceScanner(object):
    """
    Traverses directory tree, reads all source files, and passes their contents
    to the Parser.
    """

    def ScanDir(self, srcdir, parser, excluded_airframes=None):
        """
        Scans provided path and passes all found contents to the parser using
        parser.Parse method.
        """
        if excluded_airframes is not None:
            excluded_airframes = set(excluded_airframes)

        extensions = tuple(parser.GetSupportedExtensions())
        for dirname, dirnames, filenames in os.walk(srcdir):
            for filename in filenames:
                # Skip excluded airframes if provided
                if excluded_airframes and filename in excluded_airframes:
                    continue

                extension = os.path.splitext(filename)[1]
                if extension in extensions:
                    path = os.path.join(dirname, filename)
                    if not self.ScanFile(path, parser):
                        return False
        return True

    def ScanFile(self, path, parser):
        """
        Scans provided file and passes its contents to the parser using
        parser.Parse method.
        """
        with codecs.open(path, 'r', 'utf-8') as f:
            try:
                contents = f.read()
            except:
                contents = ''
                print('Failed reading file: %s, skipping content.' % path)
                pass
        return parser.Parse(path, contents)
