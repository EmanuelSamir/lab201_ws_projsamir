# Bitacora 09 set

* Existen 3 problemas principales:

  * Existe un ruido que hace que la red neuronal haga over fitting. Por lo tanto, en la salida del sistema, debe haber un filtro pasabajos para eliminar estas variaciones. De igual manera, para el entrenamiento, debe pasarse los datos por un filtro pasabajos.
  * La forma de obtención de datos es importante. En el caso **static**, la velocidad no es tan alta, y la constante de proporción de la posición, hace que no se note diferencia; sin embargo, en otros casos demuestra ser importante la velocidad. Es por ello que posiblemente el caso de  :

  $$
  q_{k+1} = q_k + \Delta q, \Delta q = NN(e, q)
  $$

  	Por ello, se requieren métodos, que bien incluyan a ambos, o separados, y luego juntarlos.

  * Se tienen que evitar las fronteras. 
  * Extras:
    * Es posible que el delay para el sistema sea un problema. 
    * Recomendación: Para el caso de ecuación sinusoidal creciente, se puede optimizar frecuencias mediante MATLAB. 