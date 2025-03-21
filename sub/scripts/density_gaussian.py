import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal

def density_gaussian(mu, xi, yi, **kwargs):
    pts = np.concatenate((xi ,yi) ,axis=1)
    cov = np.array([[1, 0], [0, 1]])
    weight = kwargs.get('weight', np.ones(mu.shape[0]) / mu.shape[0])  # e.g. np.array([1/3, 1/3, 1/3]) if len(mu) == 3
    pdf_components = np.array([multivariate_normal.pdf(pts, mean=mean, cov=cov) for mean in mu])
    fai = np.sum(pdf_components * weight[:, np.newaxis, np.newaxis], axis=0).T
    # ---------------------------------------------------------------------------#
    # 画高斯混合模型
    # ---------------------------------------------------------------------------#
    # 生成概率密度函数的x轴和y轴数据
    # x = np.linspace(-10, 10, 100)
    # y = np.linspace(-10, 10, 100)
    # X_grid, Y_grid = np.meshgrid(x, y)
    # xy = np.column_stack([X_grid.ravel(), Y_grid.ravel()])
    #
    # # 计算概率密度函数的z轴数据
    # Z = np.exp(gmm.score_samples(xy))
    # Z = Z.reshape(X_grid.shape)
    # # 绘制高斯混合模型图像
    # plt.scatter(X[:, 0], X[:, 1], alpha=0.5, color='gray')
    # plt.contour(X_grid, Y_grid, Z, levels=10, colors='red')
    # plt.xlabel('x')
    # plt.ylabel('y')
    # plt.title('Gaussian Mixture Model')
    # plt.show()
    return fai
