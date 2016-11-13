"""Just a template for subclassing"""
import os
from abc import abstractmethod, ABCMeta
import logging
from os.path import join, dirname, relpath, basename, realpath
from itertools import groupby
from jinja2 import FileSystemLoader
from jinja2.environment import Environment
import copy

from tools.targets import TARGET_MAP


class TargetNotSupportedException(Exception):
    """Indicates that an IDE does not support a particular MCU"""
    pass

class ExporterTargetsProperty(object):
    """ Exporter descriptor for TARGETS
    TARGETS as class attribute for backward compatibility
    (allows: if in Exporter.TARGETS)
    """
    def __init__(self, func):
        self.func = func
    def __get__(self, inst, cls):
        return self.func(cls)

class Exporter(object):
    """Exporter base class

    This class is meant to be extended by individual exporters, and provides a
    few helper methods for implementing an exporter with either jinja2 or
    progen.
    """
    __metaclass__ = ABCMeta
    TEMPLATE_DIR = dirname(__file__)
    DOT_IN_RELATIVE_PATH = False
    NAME = None
    TARGETS = None
    TOOLCHAIN = None


    def __init__(self, target, export_dir, project_name, toolchain,
                 extra_symbols=None, resources=None):
        """Initialize an instance of class exporter
        Positional arguments:
        target        - the target mcu/board for this project
        export_dir    - the directory of the exported project files
        project_name  - the name of the project
        toolchain     - an instance of class toolchain

        Keyword arguments:
        extra_symbols - a list of extra macros for the toolchain
        resources     - an instance of class Resources
        """
        self.export_dir = export_dir
        self.target = target
        self.project_name = project_name
        self.toolchain = toolchain
        jinja_loader = FileSystemLoader(os.path.dirname(os.path.abspath(__file__)))
        self.jinja_environment = Environment(loader=jinja_loader)
        self.resources = resources
        self.generated_files = [join(self.TEMPLATE_DIR,"GettingStarted.html")]
        self.builder_files_dict = {}
        self.add_config()

    def get_toolchain(self):
        """A helper getter function that we should probably eliminate"""
        return self.TOOLCHAIN

    def add_config(self):
        """Add the containgin directory of mbed_config.h to include dirs"""
        config = self.toolchain.get_config_header()
        if config:
            self.resources.inc_dirs.append(
                dirname(relpath(config,
                                self.resources.file_basepath[config])))

    @property
    def flags(self):
        """Returns a dictionary of toolchain flags.
        Keys of the dictionary are:
        cxx_flags    - c++ flags
        c_flags      - c flags
        ld_flags     - linker flags
        asm_flags    - assembler flags
        common_flags - common options
        """
        config_header = self.toolchain.get_config_header()
        flags = {key + "_flags": copy.deepcopy(value) for key, value
                 in self.toolchain.flags.iteritems()}
        asm_defines = ["-D" + symbol for symbol in self.toolchain.get_symbols(True)]
        c_defines = ["-D" + symbol for symbol in self.toolchain.get_symbols()]
        flags['asm_flags'] += asm_defines
        flags['c_flags'] += c_defines
        flags['cxx_flags'] += c_defines
        if config_header:
            config_header = relpath(config_header,
                                    self.resources.file_basepath[config_header])
            flags['c_flags'] += self.toolchain.get_config_option(config_header)
            flags['cxx_flags'] += self.toolchain.get_config_option(
                config_header)
        return flags

    def get_source_paths(self):
        """Returns a list of the directories where source files are contained"""
        source_keys = ['s_sources', 'c_sources', 'cpp_sources', 'hex_files',
                       'objects', 'libraries']
        source_files = []
        for key in source_keys:
            source_files.extend(getattr(self.resources, key))
        return list(set([os.path.dirname(src) for src in source_files]))

    def gen_file(self, template_file, data, target_file):
        """Generates a project file from a template using jinja"""
        jinja_loader = FileSystemLoader(
            os.path.dirname(os.path.abspath(__file__)))
        jinja_environment = Environment(loader=jinja_loader)

        template = jinja_environment.get_template(template_file)
        target_text = template.render(data)

        target_path = join(self.export_dir, target_file)
        logging.debug("Generating: %s", target_path)
        open(target_path, "w").write(target_text)
        self.generated_files += [target_path]

    def make_key(self, src):
        """From a source file, extract group name
        Positional Arguments:
        src - the src's location
        """
        key = basename(dirname(src))
        if key == ".":
            key = basename(realpath(self.export_dir))
        return key

    def group_project_files(self, sources):
        """Group the source files by their encompassing directory
        Positional Arguments:
        sources - array of source locations

        Returns a dictionary of {group name: list of source locations}
        """
        data = sorted(sources, key=self.make_key)
        return {k: list(g) for k,g in groupby(data, self.make_key)}

    @staticmethod
    def build(project_name, log_name='build_log.txt', cleanup=True):
        """Invoke exporters build command within a subprocess.
        This method is assumed to be executed at the same level as exporter
        project files and project source code.
        See uvision/__init__.py, iar/__init__.py, and makefile/__init__.py for
        example implemenation.

        Positional Arguments:
        project_name - the name of the project to build; often required by
        exporter's build command.

        Keyword Args:
        log_name - name of the build log to create. Written and printed out,
        deleted if cleanup = True
        cleanup - a boolean dictating whether exported project files and
        build log are removed after build

        Returns -1 on failure and 0 on success
        """
        raise NotImplemented("Implement in derived Exporter class.")

    @abstractmethod
    def generate(self):
        """Generate an IDE/tool specific project file"""
        raise NotImplemented("Implement a generate function in Exporter child class")
