#!/usr/bin/env python

import sys
import vtk

if __name__ == '__main__':

    in_filename = sys.argv[1]
    out_filename = sys.argv[2]

    # Read the vtk file
    reader = vtk.vtkPolyDataReader()
    reader.SetFileName(in_filename)
    reader.Update()

    # Write the stl file to disk
    writer = vtk.vtkSTLWriter()
    writer.SetFileName(out_filename)
    writer.SetInputConnection(reader.GetOutputPort())
    writer.SetFileTypeToBinary()
    writer.Write()
