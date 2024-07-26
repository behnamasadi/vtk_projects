## vtkMaskPoints 

`vtkMaskPoints` is a filter that passes through points and point attributes from input dataset. (Other geometry is not passed through.) It is possible to mask every `nth` point

```cpp  
  vtkNew<vtkPointSource> pointsSource;
  pointsSource->SetNumberOfPoints(40);

  // Create a point set
  vtkNew<vtkMaskPoints> maskPoints;
  int n=2;
  maskPoints->SetOnRatio(n); // keep every n'th point 
  maskPoints->SetInputConnection(pointsSource->GetOutputPort());
  maskPoints->Update();
```  

[code](../src/MaskPoints.cpp)
