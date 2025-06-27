#include <iostream>
#include <Kokkos_Core.hpp>

int main(int argc, char* argv[]) {
    // Initialize Kokkos
    Kokkos::initialize(argc, argv);
    
    {
        // Print configuration information
        std::cout << "=== Kokkos Configuration ===" << std::endl;
        std::cout << "Default execution space: " << Kokkos::DefaultExecutionSpace::name() << std::endl;
        std::cout << "Default host execution space: " << Kokkos::DefaultHostExecutionSpace::name() << std::endl;
        std::cout << "Concurrency: " << Kokkos::DefaultExecutionSpace::concurrency() << std::endl;
        std::cout << "=============================" << std::endl;
        
        // Create test arrays
        const int N = 1000000;
        Kokkos::View<double*> a("a", N);
        Kokkos::View<double*> b("b", N);
        Kokkos::View<double*> c("c", N);
        
        // Initialize arrays
        Kokkos::parallel_for("init", N, KOKKOS_LAMBDA(int i) {
            a(i) = 1.0;
            b(i) = 2.0;
        });
        
        // Vector addition
        auto start = std::chrono::high_resolution_clock::now();
        Kokkos::parallel_for("vector_add", N, KOKKOS_LAMBDA(int i) {
            c(i) = a(i) + b(i);
        });
        Kokkos::fence();
        auto end = std::chrono::high_resolution_clock::now();
        
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        
        // Verify result on host
        auto h_c = Kokkos::create_mirror_view(c);
        Kokkos::deep_copy(h_c, c);
        
        bool correct = true;
        for (int i = 0; i < std::min(10, N); i++) {
            if (std::abs(h_c(i) - 3.0) > 1e-10) {
                correct = false;
                break;
            }
        }
        
        std::cout << "Vector addition of " << N << " elements:" << std::endl;
        std::cout << "Time: " << duration.count() << " microseconds" << std::endl;
        std::cout << "Result: " << (correct ? "CORRECT" : "INCORRECT") << std::endl;
        std::cout << "First few results: ";
        for (int i = 0; i < std::min(5, N); i++) {
            std::cout << h_c(i) << " ";
        }
        std::cout << std::endl;
    }
    
    // Finalize Kokkos
    Kokkos::finalize();
    
    return 0;
}