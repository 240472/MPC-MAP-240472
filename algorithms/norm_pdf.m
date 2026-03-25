function [npdf] = norm_pdf(x, mu, sigma)

npdf = (1/(sigma*sqrt(2*pi))) * exp(-0.5*((x - mu)/sigma).^2);
npdf = npdf/sum(npdf);

end

