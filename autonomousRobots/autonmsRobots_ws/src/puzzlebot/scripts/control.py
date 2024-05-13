
class Control():
    def __init__(self, qd) -> None:
        self.qd = qd
        
    def controlCalculation(self, q, D_inv):
        e = self.qd -q 

        u = D_inv @ (0.1 * e)

        return u, e